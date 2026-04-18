use mr60bha2_proto::ParseEvent;
use serde::Serialize;
use std::path::Path;
use std::sync::Arc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};
use tokio::io::AsyncWriteExt;
use tokio::net::UnixListener;
use tokio::sync::broadcast;
use tracing::{debug, error, info, warn};

// ── Output types (common radar schema) ───────────────────────────────────────

#[derive(Debug, Serialize)]
struct TargetJson {
    x: f32,
    y: f32,
    speed: f32,
    dist: f32,
    angle: f32,
}

#[derive(Debug, Serialize)]
struct VitalsJson {
    heart_rate: f32,
    breath_rate: f32,
    heart_phase: f32,
    breath_phase: f32,
}

#[derive(Debug, Serialize)]
struct SnapshotJson {
    ts: f64,
    targets: Vec<TargetJson>,
    #[serde(skip_serializing_if = "Option::is_none")]
    vitals: Option<VitalsJson>,
    presence: bool,
}

// ── Aggregated sensor state ───────────────────────────────────────────────────

/// Accumulates individual frame events into a coherent snapshot.
///
/// The MR60BHA2 sends multiple frame types (positions, vitals, presence) at
/// ~8 Hz each. This struct holds the latest known value for every field and
/// emits a unified JSON snapshot on a configurable timer.
#[derive(Debug, Default)]
struct State {
    targets: Vec<TargetJson>,
    heart_rate: f32,
    breath_rate: f32,
    heart_phase: f32,
    breath_phase: f32,
    has_vitals: bool,
    presence: bool,
}

impl State {
    fn update(&mut self, event: ParseEvent) {
        match event {
            ParseEvent::PointCloudTargets(frame) => {
                self.targets = frame
                    .active_targets()
                    .map(|t| {
                        // Sanitise: replace NaN/inf before JSON serialisation
                        let x = if t.x.is_finite() { t.x } else { 0.0 };
                        let y = if t.y.is_finite() { t.y } else { 0.0 };
                        let speed = if t.speed_ms().is_finite() {
                            t.speed_ms()
                        } else {
                            0.0
                        };
                        let dist = if t.dist_m().is_finite() {
                            t.dist_m()
                        } else {
                            0.0
                        };
                        let angle = if t.angle_deg().is_finite() {
                            t.angle_deg()
                        } else {
                            0.0
                        };
                        TargetJson {
                            x,
                            y,
                            speed,
                            dist,
                            angle,
                        }
                    })
                    .collect();
            }
            ParseEvent::BreathRate(bpm) => {
                self.breath_rate = if bpm.is_finite() { bpm } else { 0.0 };
                self.has_vitals = true;
            }
            ParseEvent::HeartRate(bpm) => {
                self.heart_rate = if bpm.is_finite() { bpm } else { 0.0 };
                self.has_vitals = true;
            }
            ParseEvent::HeartBreathPhase {
                total_phase: _,
                breath_phase,
                heart_phase,
            } => {
                self.breath_phase = if breath_phase.is_finite() {
                    breath_phase
                } else {
                    0.0
                };
                self.heart_phase = if heart_phase.is_finite() {
                    heart_phase
                } else {
                    0.0
                };
                self.has_vitals = true;
            }
            ParseEvent::HumanDetected(v) => {
                self.presence = v;
            }
            // PointCloudDetection, Distance, AltPosition, StatusCode, Unknown
            // are not exposed in the common schema output.
            _ => {}
        }
    }

    fn to_json_line(&self) -> Vec<u8> {
        let ts = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs_f64();

        let vitals = if self.has_vitals {
            Some(VitalsJson {
                heart_rate: self.heart_rate,
                breath_rate: self.breath_rate,
                heart_phase: self.heart_phase,
                breath_phase: self.breath_phase,
            })
        } else {
            None
        };

        let snap = SnapshotJson {
            ts,
            targets: self
                .targets
                .iter()
                .map(|t| TargetJson {
                    x: t.x,
                    y: t.y,
                    speed: t.speed,
                    dist: t.dist,
                    angle: t.angle,
                })
                .collect(),
            vitals,
            presence: self.presence,
        };

        let mut buf = serde_json::to_vec(&snap).expect("serialization cannot fail");
        buf.push(b'\n');
        buf
    }
}

// ── Public entry point ────────────────────────────────────────────────────────

pub async fn run(
    socket_path: &Path,
    events_tx: broadcast::Sender<ParseEvent>,
    window_ms: u64,
) -> anyhow::Result<()> {
    // Remove stale socket file
    let _ = std::fs::remove_file(socket_path);

    if let Some(parent) = socket_path.parent() {
        std::fs::create_dir_all(parent)?;
    }

    let listener = UnixListener::bind(socket_path)?;

    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        std::fs::set_permissions(socket_path, std::fs::Permissions::from_mode(0o666))?;
    }

    info!(?socket_path, "listening for connections");

    // Channel carrying serialised JSON lines to connected clients.
    let (snap_tx, _) = broadcast::channel::<Arc<[u8]>>(16);

    // Aggregation task: collects ParseEvents, emits snapshots on a timer.
    let snap_tx2 = snap_tx.clone();
    let mut events = events_tx.subscribe();
    let window = Duration::from_millis(window_ms);
    tokio::spawn(async move {
        let mut state = State::default();
        let mut tick = tokio::time::interval(window);
        tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);

        loop {
            tokio::select! {
                _ = tick.tick() => {
                    let line: Arc<[u8]> = state.to_json_line().into();
                    // Ignore errors — means no clients are connected
                    let _ = snap_tx2.send(line);
                }
                result = events.recv() => {
                    match result {
                        Ok(event) => state.update(event),
                        Err(broadcast::error::RecvError::Lagged(n)) => {
                            warn!(n, "aggregator lagged, dropped parse events");
                        }
                        Err(broadcast::error::RecvError::Closed) => break,
                    }
                }
            }
        }
    });

    // Accept loop: hand each new client a receiver on the snapshot channel.
    loop {
        match listener.accept().await {
            Ok((stream, _addr)) => {
                let rx = snap_tx.subscribe();
                tokio::spawn(handle_client(stream, rx));
                debug!("new client connected");
            }
            Err(e) => {
                error!(%e, "accept error");
            }
        }
    }
}

async fn handle_client(mut stream: tokio::net::UnixStream, mut rx: broadcast::Receiver<Arc<[u8]>>) {
    loop {
        match rx.recv().await {
            Ok(line) => {
                if let Err(e) = stream.write_all(&line).await {
                    debug!(%e, "client disconnected");
                    return;
                }
            }
            Err(broadcast::error::RecvError::Lagged(n)) => {
                warn!(skipped = n, "client lagging, dropped snapshots");
            }
            Err(broadcast::error::RecvError::Closed) => {
                debug!("broadcast channel closed");
                return;
            }
        }
    }
}
