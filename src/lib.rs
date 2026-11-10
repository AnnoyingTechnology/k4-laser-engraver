//! Linux driver for K4-class USB mini laser engravers.
//!
//! Layers, bottom-up: [`protocol`] (byte frames) → [`transport`] (exclusive
//! serial port / simulator) → [`machine`] (ACK, resend, stop, framing, jobs);
//! [`sources`] + [`dxf`] (artwork) → [`job`] (mm model, TOML) → [`plan`]
//! (machine pixels); [`preview`] replays a byte stream into what would burn.

pub mod config;
pub mod dxf;
pub mod gui;
pub mod job;
pub mod machine;
pub mod plan;
pub mod preview;
pub mod protocol;
pub mod sources;
pub mod transport;
