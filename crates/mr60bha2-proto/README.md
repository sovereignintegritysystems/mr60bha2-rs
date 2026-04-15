# mr60bha2-proto

Zero-allocation protocol parser for the **Seeed MR60BHA2** 60 GHz mmWave radar sensor.

Part of the [mr60bha2-rs](https://github.com/duramson/mr60bha2-rs) project.

## Features

- Streaming frame parser (state machine, no heap allocation)
- Emits typed events for all 9 known frame types
- `no_std`-compatible (uses `libm` for math)
- Optional `std` feature for `feed_slice()` convenience method

## Usage

```rust
use mr60bha2_proto::{FrameParser, ParseEvent};

let mut parser = FrameParser::new();

// Feed bytes from UART one at a time
for &byte in uart_data {
    if let Some(event) = parser.feed(byte) {
        match event {
            ParseEvent::PointCloudTargets(frame) => {
                for target in frame.active_targets() {
                    println!(
                        "x={:.2}m y={:.2}m speed={:.2}m/s dist={:.2}m angle={:.1}°",
                        target.x,
                        target.y,
                        target.speed_ms(),
                        target.dist_m(),
                        target.angle_deg(),
                    );
                }
            }
            ParseEvent::HeartRate(bpm) => println!("heart rate: {bpm:.0} bpm"),
            ParseEvent::BreathRate(bpm) => println!("breath rate: {bpm:.0} bpm"),
            ParseEvent::HumanDetected(present) => println!("presence: {present}"),
            _ => {}
        }
    }
}
```

## Events

| Variant | Frame type | Description |
|---|---|---|
| `PointCloudTargets` | `0x0A04` | Up to 3 position targets (x, y metres; Doppler index) |
| `PointCloudDetection` | `0x0A08` | Detection point cloud (same payload, different context) |
| `HeartBreathPhase` | `0x0A13` | Phase values for respiration and heartbeat |
| `BreathRate` | `0x0A14` | Breathing rate in breaths/min |
| `HeartRate` | `0x0A15` | Heart rate in beats/min |
| `Distance` | `0x0A16` | Sensor-reported distance measurement |
| `HumanDetected` | `0x0F09` | Binary presence/absence detection |
| `AltPosition` | `0x0A17` | Alternate position frame (undocumented) |
| `StatusCode` | `0x0A29` | Status/error code (undocumented) |
| `Unknown` | — | Any unrecognised frame type |

## Protocol

Frames use a binary framing with a 9-byte header:

```
SOF (1)  │ 0x53
Header   │ ID(2 BE) + LEN(2 BE) + TYPE(2 BE) + HEAD_CKSUM(1) + DATA_CKSUM(1)
Data     │ LEN bytes, little-endian payload
```

Checksum: `~(XOR of covered bytes)` — one's complement of XOR over the covered range.

Up to **3 targets** tracked simultaneously at ~8 Hz.

## no_std

`mr60bha2-proto` is `no_std` by default. Add it to your embedded project:

```toml
[dependencies]
mr60bha2-proto = { version = "0.1", default-features = false }
```

The `std` feature enables the `feed_slice(&[u8]) -> Vec<ParseEvent>` helper:

```toml
[dependencies]
mr60bha2-proto = { version = "0.1", features = ["std"] }
```

## License

Licensed under either of [Apache License, Version 2.0](../../LICENSE-APACHE)
or [MIT License](../../LICENSE-MIT) at your option.
