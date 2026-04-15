use mr60bha2_proto::{FrameParser, ParseEvent};
use tokio::sync::broadcast;
use tokio_serial::SerialPortBuilderExt;
use tracing::{debug, info, warn};

pub async fn run(
    device: std::path::PathBuf,
    baud_rate: u32,
    tx: broadcast::Sender<ParseEvent>,
) -> anyhow::Result<()> {
    info!(?device, baud_rate, "opening serial port");

    let mut port = tokio_serial::new(device.to_string_lossy(), baud_rate)
        .data_bits(tokio_serial::DataBits::Eight)
        .stop_bits(tokio_serial::StopBits::One)
        .parity(tokio_serial::Parity::None)
        .open_native_async()?;

    #[cfg(target_os = "linux")]
    {
        port.set_exclusive(false)?;
    }

    info!("serial port opened, starting frame parser");

    let mut parser = FrameParser::new();
    let mut buf = [0u8; 512];

    use tokio::io::AsyncReadExt;
    loop {
        let n = port.read(&mut buf).await?;
        if n == 0 {
            warn!("serial port returned 0 bytes, EOF?");
            tokio::time::sleep(std::time::Duration::from_millis(100)).await;
            continue;
        }

        debug!(bytes = n, "read from serial");

        for &byte in &buf[..n] {
            if let Some(event) = parser.feed(byte) {
                debug!(?event, "parsed frame");
                // Ignore send errors — means no receivers are connected yet
                let _ = tx.send(event);
            }
        }
    }
}
