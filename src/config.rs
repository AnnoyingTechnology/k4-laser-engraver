//! User configuration: `$XDG_CONFIG_HOME/k4laser/config.toml`.
//!
//! ```toml
//! port = "/dev/serial/by-path/pci-0000:00:14.0-usb-0:2:1.0-port0"
//! [settings]          # 0x28 payload; omitted keys keep the vendor defaults
//! standby_intensity = 65
//! ```

use std::path::PathBuf;

use anyhow::{Result, anyhow};
use serde::{Deserialize, Serialize};

use crate::protocol::MachineSettings;

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct Config {
    /// Serial device; use the stable /dev/serial/by-path name.
    pub port: Option<PathBuf>,
    pub settings: MachineSettings,
}

pub fn path() -> PathBuf {
    let base = std::env::var_os("XDG_CONFIG_HOME")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".config")))
        .unwrap_or_else(|| PathBuf::from("."));
    base.join("k4laser").join("config.toml")
}

pub fn load() -> Result<Config> {
    let p = path();
    match std::fs::read_to_string(&p) {
        Ok(text) => toml::from_str(&text).map_err(|e| anyhow!("{}: {e}", p.display())),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(Config::default()),
        Err(e) => Err(anyhow!("{}: {e}", p.display())),
    }
}

pub fn save(cfg: &Config) -> Result<()> {
    let p = path();
    if let Some(dir) = p.parent() {
        std::fs::create_dir_all(dir)?;
    }
    let tmp = p.with_extension("toml.tmp");
    std::fs::write(&tmp, toml::to_string_pretty(cfg)?)?;
    std::fs::rename(&tmp, &p)?;
    Ok(())
}
