use serde::Deserialize;
use std::path::PathBuf;

#[derive(Debug, Deserialize)]
pub struct Config {
    #[serde(default = "default_device")]
    pub device: PathBuf,
    #[serde(default = "default_baud_rate")]
    pub baud_rate: u32,
    #[serde(default = "default_socket_path")]
    pub socket_path: PathBuf,
    /// Snapshot emission interval in milliseconds (default: 125 ms = ~8 Hz).
    #[serde(default = "default_window_ms")]
    pub window_ms: u64,
    #[serde(default = "default_log_level")]
    pub log_level: String,
}

fn default_device() -> PathBuf {
    PathBuf::from("/dev/ttyAMA0")
}

fn default_baud_rate() -> u32 {
    115200
}

fn default_socket_path() -> PathBuf {
    PathBuf::from("/run/mr60bha2/radar.sock")
}

fn default_window_ms() -> u64 {
    125
}

fn default_log_level() -> String {
    "info".to_string()
}

impl Default for Config {
    fn default() -> Self {
        Self {
            device:      default_device(),
            baud_rate:   default_baud_rate(),
            socket_path: default_socket_path(),
            window_ms:   default_window_ms(),
            log_level:   default_log_level(),
        }
    }
}

impl Config {
    pub fn load(path: &std::path::Path) -> anyhow::Result<Self> {
        let content = std::fs::read_to_string(path)?;
        let config: Config = toml::from_str(&content)?;
        Ok(config)
    }
}
