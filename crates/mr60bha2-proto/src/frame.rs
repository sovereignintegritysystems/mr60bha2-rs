use crate::types::PointCloudFrame;

/// Maximum data section length per the MR60BHA2 protocol.
const MAX_DATA_LEN: usize = 256;

/// SOF marker byte.
const SOF: u8 = 0x01;

/// Events emitted by the frame parser.
///
/// Each variant corresponds to one physical UART frame from the sensor.
/// The sensor transmits all frame types simultaneously at ~8 Hz.
#[derive(Debug, Clone, PartialEq)]
pub enum ParseEvent {
    /// `0x0A04` — Primary tracked targets with position, Doppler, and cluster ID.
    PointCloudTargets(PointCloudFrame),
    /// `0x0A08` — Detection-phase point cloud (structure identical; often empty).
    PointCloudDetection(PointCloudFrame),
    /// `0x0A13` — Heart and breath phases in radians.
    HeartBreathPhase {
        total_phase: f32,
        breath_phase: f32,
        heart_phase: f32,
    },
    /// `0x0A14` — Breath rate in breaths per minute.
    BreathRate(f32),
    /// `0x0A15` — Heart rate in beats per minute.
    HeartRate(f32),
    /// `0x0A16` — Distance measurement (metres) and range validity flag.
    Distance { value: f32, valid: bool },
    /// `0x0F09` — Human presence detection.
    HumanDetected(bool),
    /// `0x0A17` — Undocumented: alternative/unfiltered target position (metres).
    AltPosition { x: f32, y: f32 },
    /// `0x0A29` — Undocumented: status/tracking-phase indicator.
    StatusCode(u16),
    /// Any unrecognised frame type — passed through for debugging.
    Unknown { frame_type: u16 },
}

/// Parser states.
#[derive(Debug, Clone, Copy)]
enum State {
    /// Scanning for the SOF byte (0x01).
    SearchSof,
    /// Reading the 7 bytes after SOF: ID(2), LEN(2), TYPE(2), HEAD_CKSUM(1).
    ReadHeader { pos: u8 },
    /// Reading data bytes; `remaining` counts bytes still to read.
    ReadData { remaining: u16 },
    /// Reading the single data checksum byte.
    ReadDataCksum,
}

/// Streaming frame parser for the MR60BHA2 60 GHz mmWave sensor.
///
/// Feed bytes one at a time via [`feed`]. The parser emits a [`ParseEvent`]
/// for each complete, checksum-validated frame in the byte stream.
///
/// This is a zero-allocation state machine suitable for `no_std` environments.
///
/// # Frame format
///
/// ```text
/// [SOF:1] [ID:2] [LEN:2] [TYPE:2] [HEAD_CKSUM:1] [DATA:LEN] [DATA_CKSUM:1]
/// ```
///
/// All header multi-byte fields (LEN, TYPE) are **big-endian**.
/// All data-section fields are **little-endian** (native ARM byte order).
///
/// Checksum formula: `~(XOR of covered bytes) & 0xFF`
#[derive(Debug)]
pub struct FrameParser {
    state: State,
    /// Header bytes 1–7 after SOF: ID[2], LEN[2], TYPE[2], HEAD_CKSUM[1]
    header: [u8; 7],
    data_len: u16,
    frame_type: u16,
    data: [u8; MAX_DATA_LEN],
    data_pos: usize,
    data_xor: u8,
}

impl FrameParser {
    pub fn new() -> Self {
        Self {
            state: State::SearchSof,
            header: [0u8; 7],
            data_len: 0,
            frame_type: 0,
            data: [0u8; MAX_DATA_LEN],
            data_pos: 0,
            data_xor: 0,
        }
    }

    /// Feed a single byte. Returns `Some(ParseEvent)` when a complete valid
    /// frame has been assembled and both checksums pass.
    #[inline]
    pub fn feed(&mut self, byte: u8) -> Option<ParseEvent> {
        match self.state {
            State::SearchSof => {
                if byte == SOF {
                    self.state = State::ReadHeader { pos: 0 };
                }
                None
            }

            State::ReadHeader { pos } => {
                self.header[pos as usize] = byte;
                let next = pos + 1;
                if next < 7 {
                    self.state = State::ReadHeader { pos: next };
                    return None;
                }

                // All 7 post-SOF header bytes collected.
                // Validate HEAD_CKSUM = ~(XOR of SOF + header[0..6]).
                let raw_xor = self.header[..6].iter()
                    .fold(SOF, |acc, &b| acc ^ b);
                if self.header[6] != !raw_xor {
                    self.state = State::SearchSof;
                    return None;
                }

                // LEN and TYPE are big-endian.
                let len = u16::from_be_bytes([self.header[2], self.header[3]]);
                if len as usize > MAX_DATA_LEN {
                    self.state = State::SearchSof;
                    return None;
                }

                self.data_len = len;
                self.frame_type = u16::from_be_bytes([self.header[4], self.header[5]]);
                self.data_pos = 0;
                self.data_xor = 0;

                if len == 0 {
                    self.state = State::ReadDataCksum;
                } else {
                    self.state = State::ReadData { remaining: len };
                }
                None
            }

            State::ReadData { remaining } => {
                self.data[self.data_pos] = byte;
                self.data_pos += 1;
                self.data_xor ^= byte;
                if remaining == 1 {
                    self.state = State::ReadDataCksum;
                } else {
                    self.state = State::ReadData { remaining: remaining - 1 };
                }
                None
            }

            State::ReadDataCksum => {
                self.state = State::SearchSof;
                if byte == !self.data_xor {
                    self.dispatch()
                } else {
                    None
                }
            }
        }
    }

    /// Feed a slice of bytes, collecting all parsed events.
    /// For performance-critical paths prefer calling [`feed`] in a loop.
    #[cfg(feature = "std")]
    pub fn feed_slice(&mut self, data: &[u8]) -> Vec<ParseEvent> {
        let mut events = Vec::new();
        for &b in data {
            if let Some(ev) = self.feed(b) {
                events.push(ev);
            }
        }
        events
    }

    /// Dispatch a validated frame to a `ParseEvent`.
    fn dispatch(&self) -> Option<ParseEvent> {
        let data = &self.data[..self.data_len as usize];
        let ft = self.frame_type;

        let event = match ft {
            0x0A04 => ParseEvent::PointCloudTargets(PointCloudFrame::from_bytes(data)),
            0x0A08 => ParseEvent::PointCloudDetection(PointCloudFrame::from_bytes(data)),

            0x0A13 if data.len() >= 12 => ParseEvent::HeartBreathPhase {
                total_phase:  le_f32(&data[0..]),
                breath_phase: le_f32(&data[4..]),
                heart_phase:  le_f32(&data[8..]),
            },

            0x0A14 if data.len() >= 4 => ParseEvent::BreathRate(le_f32(data)),
            0x0A15 if data.len() >= 4 => ParseEvent::HeartRate(le_f32(data)),

            0x0A16 if data.len() >= 8 => ParseEvent::Distance {
                valid: le_u32(&data[0..]) == 1,
                value: le_f32(&data[4..]),
            },

            0x0F09 if !data.is_empty() => ParseEvent::HumanDetected(data[0] != 0),

            0x0A17 if data.len() >= 8 => ParseEvent::AltPosition {
                x: le_f32(&data[0..]),
                y: le_f32(&data[4..]),
            },

            0x0A29 if data.len() >= 2 => {
                ParseEvent::StatusCode(u16::from_le_bytes([data[0], data[1]]))
            }

            _ => ParseEvent::Unknown { frame_type: ft },
        };
        Some(event)
    }
}

impl Default for FrameParser {
    fn default() -> Self {
        Self::new()
    }
}

#[inline(always)]
fn le_f32(b: &[u8]) -> f32 {
    f32::from_le_bytes([b[0], b[1], b[2], b[3]])
}

#[inline(always)]
fn le_u32(b: &[u8]) -> u32 {
    u32::from_le_bytes([b[0], b[1], b[2], b[3]])
}

// ── Frame builder helper (for tests) ─────────────────────────────────────────

#[cfg(test)]
fn build_frame(frame_type: u16, data: &[u8]) -> Vec<u8> {
    let id: [u8; 2] = [0x80, 0x00];
    let len = (data.len() as u16).to_be_bytes();
    let ft  = frame_type.to_be_bytes();

    let head_xor = [SOF, id[0], id[1], len[0], len[1], ft[0], ft[1]]
        .iter().fold(0u8, |acc, &b| acc ^ b);
    let head_cksum = !head_xor;

    let data_xor   = data.iter().fold(0u8, |acc, &b| acc ^ b);
    let data_cksum = !data_xor;

    let mut frame = vec![SOF];
    frame.extend_from_slice(&id);
    frame.extend_from_slice(&len);
    frame.extend_from_slice(&ft);
    frame.push(head_cksum);
    frame.extend_from_slice(data);
    frame.push(data_cksum);
    frame
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── Breath rate ────────────────────────────────────────────────────────

    #[test]
    fn parse_breath_rate() {
        let data = 16.5f32.to_le_bytes();
        let raw  = build_frame(0x0A14, &data);

        let mut p = FrameParser::new();
        let events = p.feed_slice(&raw);
        assert_eq!(events.len(), 1);
        assert!(matches!(events[0], ParseEvent::BreathRate(v) if (v - 16.5).abs() < 1e-5));
    }

    // ── Heart rate ─────────────────────────────────────────────────────────

    #[test]
    fn parse_heart_rate() {
        let data = 72.0f32.to_le_bytes();
        let raw  = build_frame(0x0A15, &data);

        let mut p = FrameParser::new();
        let events = p.feed_slice(&raw);
        assert_eq!(events.len(), 1);
        assert!(matches!(events[0], ParseEvent::HeartRate(v) if (v - 72.0).abs() < 1e-5));
    }

    // ── Heart/breath phases ────────────────────────────────────────────────

    #[test]
    fn parse_heart_breath_phase() {
        let mut data = [0u8; 12];
        data[0..4].copy_from_slice(&1.0f32.to_le_bytes()); // total
        data[4..8].copy_from_slice(&0.5f32.to_le_bytes()); // breath
        data[8..12].copy_from_slice(&2.1f32.to_le_bytes()); // heart
        let raw = build_frame(0x0A13, &data);

        let mut p = FrameParser::new();
        let events = p.feed_slice(&raw);
        assert_eq!(events.len(), 1);
        match &events[0] {
            ParseEvent::HeartBreathPhase { total_phase, breath_phase, heart_phase } => {
                assert!((total_phase  - 1.0).abs() < 1e-5);
                assert!((breath_phase - 0.5).abs() < 1e-5);
                assert!((heart_phase  - 2.1).abs() < 1e-5);
            }
            _ => panic!("wrong event"),
        }
    }

    // ── Human detection ────────────────────────────────────────────────────

    #[test]
    fn parse_human_detected_true() {
        let raw = build_frame(0x0F09, &[0x01]);
        let mut p = FrameParser::new();
        let events = p.feed_slice(&raw);
        assert!(matches!(events[0], ParseEvent::HumanDetected(true)));
    }

    #[test]
    fn parse_human_detected_false() {
        let raw = build_frame(0x0F09, &[0x00]);
        let mut p = FrameParser::new();
        let events = p.feed_slice(&raw);
        assert!(matches!(events[0], ParseEvent::HumanDetected(false)));
    }

    // ── Distance ───────────────────────────────────────────────────────────

    #[test]
    fn parse_distance_valid() {
        let mut data = [0u8; 8];
        data[0..4].copy_from_slice(&1u32.to_le_bytes());    // range_flag=1 → valid
        data[4..8].copy_from_slice(&1.23f32.to_le_bytes()); // 1.23 m
        let raw = build_frame(0x0A16, &data);

        let mut p = FrameParser::new();
        let events = p.feed_slice(&raw);
        match &events[0] {
            ParseEvent::Distance { valid, value } => {
                assert!(valid);
                assert!((value - 1.23).abs() < 1e-5);
            }
            _ => panic!("wrong event"),
        }
    }

    // ── Point cloud targets ────────────────────────────────────────────────

    #[test]
    fn parse_point_cloud_one_target() {
        // count=1 + one 16-byte target
        let mut data = vec![0u8; 4 + 16];
        data[0..4].copy_from_slice(&1u32.to_le_bytes());
        // target: x=0.5m, y=1.1m, dop=-3, cid=1
        data[4..8].copy_from_slice(&0.5f32.to_le_bytes());
        data[8..12].copy_from_slice(&1.1f32.to_le_bytes());
        data[12..16].copy_from_slice(&(-3i32).to_le_bytes());
        data[16..20].copy_from_slice(&1i32.to_le_bytes());
        let raw = build_frame(0x0A04, &data);

        let mut p = FrameParser::new();
        let events = p.feed_slice(&raw);
        match &events[0] {
            ParseEvent::PointCloudTargets(f) => {
                assert_eq!(f.count, 1);
                let t = &f.targets[0];
                assert!((t.x - 0.5).abs() < 1e-5);
                assert!((t.y - 1.1).abs() < 1e-5);
                assert_eq!(t.dop_index, -3);
                assert_eq!(t.cluster_id, 1);
            }
            _ => panic!("wrong event"),
        }
    }

    #[test]
    fn parse_point_cloud_empty() {
        // count=0 → no target data
        let data = 0u32.to_le_bytes();
        let raw  = build_frame(0x0A04, &data);

        let mut p = FrameParser::new();
        let events = p.feed_slice(&raw);
        match &events[0] {
            ParseEvent::PointCloudTargets(f) => assert_eq!(f.count, 0),
            _ => panic!("wrong event"),
        }
    }

    // ── Unknown frame ──────────────────────────────────────────────────────

    #[test]
    fn parse_unknown_frame_type() {
        let raw = build_frame(0xBEEF, &[0xDE, 0xAD]);
        let mut p = FrameParser::new();
        let events = p.feed_slice(&raw);
        assert!(matches!(events[0], ParseEvent::Unknown { frame_type: 0xBEEF }));
    }

    // ── Robustness ─────────────────────────────────────────────────────────

    #[test]
    fn garbage_prefix_is_ignored() {
        let mut stream = vec![0xFF, 0x42, 0x00, 0x13]; // garbage
        let data = 14.5f32.to_le_bytes();
        stream.extend(build_frame(0x0A14, &data));

        let mut p = FrameParser::new();
        let events = p.feed_slice(&stream);
        assert_eq!(events.len(), 1);
        assert!(matches!(events[0], ParseEvent::BreathRate(v) if (v - 14.5).abs() < 1e-5));
    }

    #[test]
    fn bad_head_checksum_is_rejected() {
        let data = 16.5f32.to_le_bytes();
        let mut raw = build_frame(0x0A14, &data);
        // Corrupt the HEAD_CKSUM byte (index 7)
        raw[7] ^= 0xFF;

        let mut p = FrameParser::new();
        let events = p.feed_slice(&raw);
        assert!(events.is_empty());
    }

    #[test]
    fn bad_data_checksum_is_rejected() {
        let data = 16.5f32.to_le_bytes();
        let mut raw = build_frame(0x0A14, &data);
        // Corrupt the last byte (DATA_CKSUM)
        *raw.last_mut().unwrap() ^= 0xFF;

        let mut p = FrameParser::new();
        let events = p.feed_slice(&raw);
        assert!(events.is_empty());
    }

    #[test]
    fn multiple_frames_in_sequence() {
        let mut stream = Vec::new();
        stream.extend(build_frame(0x0A14, &16.5f32.to_le_bytes()));
        stream.extend(build_frame(0x0A15, &72.0f32.to_le_bytes()));
        stream.extend(build_frame(0x0F09, &[0x01]));

        let mut p = FrameParser::new();
        let events = p.feed_slice(&stream);
        assert_eq!(events.len(), 3);
        assert!(matches!(events[0], ParseEvent::BreathRate(_)));
        assert!(matches!(events[1], ParseEvent::HeartRate(_)));
        assert!(matches!(events[2], ParseEvent::HumanDetected(true)));
    }

    #[test]
    fn byte_by_byte_feeding() {
        let data = 16.5f32.to_le_bytes();
        let raw  = build_frame(0x0A14, &data);

        let mut p = FrameParser::new();
        let mut events = Vec::new();
        for &b in &raw {
            if let Some(ev) = p.feed(b) {
                events.push(ev);
            }
        }
        assert_eq!(events.len(), 1);
        assert!(matches!(events[0], ParseEvent::BreathRate(v) if (v - 16.5).abs() < 1e-5));
    }

    #[test]
    fn zero_length_data_frame() {
        // A frame with LEN=0 still has a DATA_CKSUM byte (~(XOR of nothing) = ~0x00 = 0xFF)
        let raw = build_frame(0x0F09, &[]);
        // 0x0F09 needs at least 1 byte → should emit Unknown, not panic
        let mut p = FrameParser::new();
        let events = p.feed_slice(&raw);
        assert_eq!(events.len(), 1);
        // With 0 data bytes, the 0x0F09 arm's guard `!data.is_empty()` fails → Unknown
        assert!(matches!(events[0], ParseEvent::Unknown { frame_type: 0x0F09 }));
    }
}
