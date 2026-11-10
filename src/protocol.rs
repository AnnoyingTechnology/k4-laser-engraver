//! K4 wire protocol.
//!
//! Reverse-engineered from the vendor application "Laser_K4ble V2.7
//! (2022-06-25)"; see PROTOCOL.md for the evidence trail.
//!
//! Transport: 115200 baud, 8N1, no flow control (CH340 bridge).
//!
//! Host -> device frames: `[opcode u8][length u16 BE][payload]`, where
//! `length` counts the whole frame. Argument-less commands are 4 bytes with a
//! zero pad: `[op, 0x00, 0x04, 0x00]`.
//!
//! Device -> host: one `0x09` (ACK) or `0x08` (resend) per command, 3 bytes
//! for the model query, `FF FF FF FE` once the offline-job flash erase is done.
//!
//! All coordinates are machine pixels of 0.05 mm (508 DPI).

use std::fmt;

pub const BAUD: u32 = 115_200;
pub const PITCH_MM: f64 = 0.05;
pub const PX_PER_MM: f64 = 1.0 / PITCH_MM;

pub const ACK: u8 = 0x09;
pub const NAK: u8 = 0x08;
pub const ERASE_DONE: [u8; 4] = [0xFF, 0xFF, 0xFF, 0xFE];

pub const OP_MOVE_REL: u8 = 0x01;
pub const OP_RESET: u8 = 0x06;
pub const OP_RASTER_ROW: u8 = 0x09;
pub const OP_HANDSHAKE: u8 = 0x0A;
pub const OP_START: u8 = 0x14;
pub const OP_STOP: u8 = 0x16;
pub const OP_HOME: u8 = 0x17;
pub const OP_PAUSE: u8 = 0x18;
pub const OP_RESUME: u8 = 0x19;
pub const OP_VECTOR: u8 = 0x1D;
pub const OP_FRAME_ON: u8 = 0x20;
pub const OP_FRAME_OFF: u8 = 0x21;
pub const OP_OFFLINE_ROW: u8 = 0x22;
pub const OP_OFFLINE_JOB: u8 = 0x23;
pub const OP_SETTINGS: u8 = 0x28;
/// Hidden handlers in V2.7 (buttons not placed on any form); purpose unconfirmed.
pub const OP_AUX_ON: u8 = 0xFD;
pub const OP_AUX_OFF: u8 = 0xFE;
pub const OP_MODEL: u8 = 0xFF;

/// The vendor sends vector data in batches of 640 coordinate bytes.
pub const VECTOR_MAX_SEGMENTS: usize = 80;
/// Per mille; the vendor UI shows 0-100 % and multiplies by 10.
pub const POWER_MAX: u16 = 1000;
/// Vendor "carving depth" slider: dwell per pixel, unit unknown.
pub const DEPTH_MAX: u16 = 100;
/// Vendor constant used to size offline jobs.
pub const OFFLINE_CHUNK: u32 = 4069;

/// Work area of the K4 (model byte 3 = 4 or 31): 1600 x 1520 px = 80 x 76 mm.
pub const K4_AREA_PX: (u32, u32) = (1600, 1520);

pub fn op_name(op: u8) -> String {
    match op {
        OP_MOVE_REL => "MOVE_REL",
        OP_RESET => "RESET",
        OP_RASTER_ROW => "RASTER_ROW",
        OP_HANDSHAKE => "HANDSHAKE",
        OP_START => "START",
        OP_STOP => "STOP",
        OP_HOME => "HOME",
        OP_PAUSE => "PAUSE",
        OP_RESUME => "RESUME",
        OP_VECTOR => "VECTOR",
        OP_FRAME_ON => "FRAME_ON",
        OP_FRAME_OFF => "FRAME_OFF",
        OP_OFFLINE_ROW => "OFFLINE_ROW",
        OP_OFFLINE_JOB => "OFFLINE_JOB",
        OP_SETTINGS => "SETTINGS",
        OP_AUX_ON => "AUX_FD",
        OP_AUX_OFF => "AUX_FE",
        OP_MODEL => "MODEL",
        _ => return format!("OP_{op:02X}"),
    }
    .to_string()
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ProtocolError(pub String);

impl fmt::Display for ProtocolError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for ProtocolError {}

pub type Result<T> = std::result::Result<T, ProtocolError>;

fn u16_field(value: i64, name: &str) -> Result<[u8; 2]> {
    u16::try_from(value).map(u16::to_be_bytes).map_err(|_| ProtocolError(format!("{name}={value} outside u16 range")))
}

fn s16_field(value: i64, name: &str) -> Result<[u8; 2]> {
    i16::try_from(value).map(i16::to_be_bytes).map_err(|_| ProtocolError(format!("{name}={value} outside s16 range")))
}

fn frame(op: u8, payload: &[u8]) -> Result<Vec<u8>> {
    let len = 3 + payload.len();
    let len16 = u16::try_from(len).map_err(|_| ProtocolError(format!("frame of {len} bytes too long")))?;
    let mut out = Vec::with_capacity(len);
    out.push(op);
    out.extend_from_slice(&len16.to_be_bytes());
    out.extend_from_slice(payload);
    Ok(out)
}

fn simple(op: u8) -> Vec<u8> {
    vec![op, 0x00, 0x04, 0x00]
}

fn check_power_depth(power: u16, depth: u16) -> Result<()> {
    if power > POWER_MAX {
        return Err(ProtocolError(format!("power={power} outside 0..{POWER_MAX} per mille")));
    }
    if depth > DEPTH_MAX {
        return Err(ProtocolError(format!("depth={depth} outside 0..{DEPTH_MAX}")));
    }
    Ok(())
}

// ------------------------------------------------------------------ commands

pub fn handshake() -> Vec<u8> {
    simple(OP_HANDSHAKE)
}
pub fn query_model() -> Vec<u8> {
    simple(OP_MODEL)
}
pub fn home() -> Vec<u8> {
    simple(OP_HOME)
}
pub fn reset() -> Vec<u8> {
    simple(OP_RESET)
}
pub fn stop() -> Vec<u8> {
    simple(OP_STOP)
}
pub fn pause() -> Vec<u8> {
    simple(OP_PAUSE)
}
pub fn resume() -> Vec<u8> {
    simple(OP_RESUME)
}
pub fn frame_off() -> Vec<u8> {
    simple(OP_FRAME_OFF)
}
pub fn aux(on: bool) -> Vec<u8> {
    simple(if on { OP_AUX_ON } else { OP_AUX_OFF })
}

/// Begin an online job whose top-left corner is at machine (x, y).
pub fn start(x: i64, y: i64) -> Result<Vec<u8>> {
    frame(OP_START, &[u16_field(x, "x")?, u16_field(y, "y")?].concat())
}

pub fn move_relative(dx: i64, dy: i64) -> Result<Vec<u8>> {
    frame(OP_MOVE_REL, &[s16_field(dx, "dx")?, s16_field(dy, "dy")?].concat())
}

/// Trace a rectangle with the low-power beam, offset (x, y) from the head position.
pub fn frame_on(width: i64, height: i64, x: i64, y: i64) -> Result<Vec<u8>> {
    let p = [u16_field(width, "width")?, u16_field(height, "height")?, u16_field(x, "x")?, u16_field(y, "y")?];
    frame(OP_FRAME_ON, &p.concat())
}

/// One raster line: `bits` is MSB-first, 1 = burn, leftmost pixel first.
pub fn raster_row(row: u16, bits: &[u8], power: u16, depth: u16) -> Result<Vec<u8>> {
    check_power_depth(power, depth)?;
    let mut p = Vec::with_capacity(6 + bits.len());
    p.extend_from_slice(&depth.to_be_bytes());
    p.extend_from_slice(&power.to_be_bytes());
    p.extend_from_slice(&row.to_be_bytes());
    p.extend_from_slice(bits);
    frame(OP_RASTER_ROW, &p)
}

/// A machine-pixel segment (x0, y0, x1, y1), relative to the job origin.
pub type Segment = [u16; 4];

/// Marker segments in the vector stream. The vendor G-code import maps `M3`
/// (laser on) to 50001 and `M5` (laser off) to 50000.
pub const VECTOR_LASER_ON: Segment = [0xC351; 4];
pub const VECTOR_LASER_OFF: Segment = [0xC350; 4];

/// The segment list of a vector job: the paths `passes` times over, each path
/// between laser-on and laser-off markers and reached by a laser-off move,
/// then a laser-off move back to the job origin. Without the markers the
/// machine burns the moves between paths.
///
/// Same layout as the vendor (`ON path OFF move ON path … OFF`), except that
/// the vendor reaches the first path with the laser on, and leaves the head
/// where the last path ends.
pub fn vector_stream(paths: &[Segment], passes: u32) -> Vec<Segment> {
    let mut out = Vec::with_capacity(vector_stream_len(paths, passes));
    let mut at = None;
    for &s in paths.iter().cycle().take(paths.len() * passes as usize) {
        if at != Some((s[0], s[1])) {
            let (x, y) = at.unwrap_or((s[0], s[1]));
            out.extend([VECTOR_LASER_OFF, [x, y, s[0], s[1]], VECTOR_LASER_ON]);
        }
        out.push(s);
        at = Some((s[2], s[3]));
    }
    if let Some((x, y)) = at {
        out.extend([VECTOR_LASER_OFF, [x, y, 0, 0]]);
    }
    out
}

/// `vector_stream(paths, passes).len()`, without building it.
pub fn vector_stream_len(paths: &[Segment], passes: u32) -> usize {
    let (Some(first), Some(last), n) = (paths.first(), paths.last(), passes as usize) else { return 0 };
    if n == 0 {
        return 0;
    }
    let breaks = paths.windows(2).filter(|w| w[0][2..] != w[1][..2]).count();
    let wrap = (last[2..] != first[..2]) as usize; // from the last path back to the first
    let starts = 1 + n * breaks + (n - 1) * wrap;
    paths.len() * n + 3 * starts + 2
}

/// Up to 80 segments relative to the job origin.
pub fn vector_batch(segments: &[Segment], power: u16, depth: u16) -> Result<Vec<u8>> {
    check_power_depth(power, depth)?;
    if segments.is_empty() || segments.len() > VECTOR_MAX_SEGMENTS {
        return Err(ProtocolError(format!("vector batch needs 1..{VECTOR_MAX_SEGMENTS} segments")));
    }
    let mut p = Vec::with_capacity(4 + 8 * segments.len());
    p.extend_from_slice(&depth.to_be_bytes());
    p.extend_from_slice(&power.to_be_bytes());
    for s in segments {
        for v in s {
            p.extend_from_slice(&v.to_be_bytes());
        }
    }
    frame(OP_VECTOR, &p)
}

/// Payload of the 0x28 frame, sent by the vendor app on every connect.
///
/// Defaults are the values shipped in the vendor `pei_zhi.ini`; names
/// translate the vendor's (Chinese) settings-dialog labels.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct MachineSettings {
    /// u8, 待机激光强度: standby (pointer/framing) laser intensity.
    pub standby_intensity: u8,
    /// u16, 行延时: line delay.
    pub line_delay: u16,
    /// u16, 最大激光功率: max laser power (PWM top).
    pub max_power: u16,
    /// u8, 细分数: stepper microstepping.
    pub microsteps: u8,
    /// u8, X轴换向补偿: X reversal (backlash) compensation.
    pub x_backlash: u8,
    /// u8, Y轴换向补偿: Y reversal (backlash) compensation.
    pub y_backlash: u8,
    /// u16, 变速阶梯数: acceleration ramp steps.
    pub accel_steps: u16,
    /// u16, 速度下限: speed lower limit (timer period).
    pub speed_min: u16,
    /// u16, 速度上限: speed upper limit (timer period).
    pub speed_max: u16,
    /// u16, 框定位速度: framing speed.
    pub frame_speed: u16,
}

impl Default for MachineSettings {
    fn default() -> Self {
        Self {
            standby_intensity: 65,
            line_delay: 100,
            max_power: 1000,
            microsteps: 8,
            x_backlash: 1,
            y_backlash: 1,
            accel_steps: 200,
            speed_min: 650,
            speed_max: 400,
            frame_speed: 10,
        }
    }
}

impl MachineSettings {
    pub fn encode(&self) -> Vec<u8> {
        let mut p = Vec::with_capacity(16);
        p.push(self.standby_intensity);
        p.extend_from_slice(&self.line_delay.to_be_bytes());
        p.extend_from_slice(&self.max_power.to_be_bytes());
        p.push(self.microsteps);
        p.push(self.x_backlash);
        p.push(self.y_backlash);
        p.extend_from_slice(&self.accel_steps.to_be_bytes());
        p.extend_from_slice(&self.speed_min.to_be_bytes());
        p.extend_from_slice(&self.speed_max.to_be_bytes());
        p.extend_from_slice(&self.frame_speed.to_be_bytes());
        frame(OP_SETTINGS, &p).expect("settings frame is 19 bytes")
    }

    fn decode(payload: &[u8]) -> Option<Self> {
        if payload.len() < 16 {
            return None;
        }
        let u = |i: usize| u16::from_be_bytes([payload[i], payload[i + 1]]);
        Some(Self {
            standby_intensity: payload[0],
            line_delay: u(1),
            max_power: u(3),
            microsteps: payload[5],
            x_backlash: payload[6],
            y_backlash: payload[7],
            accel_steps: u(8),
            speed_min: u(10),
            speed_max: u(12),
            frame_speed: u(14),
        })
    }
}

/// 3-byte reply to the model query; the vendor keys the work area on byte 3.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Model(pub [u8; 3]);

impl Model {
    pub fn code(&self) -> String {
        format!("{},{},{}", self.0[0], self.0[1], self.0[2])
    }

    pub fn area_px(&self) -> Option<(u32, u32)> {
        match self.0[2] {
            4 | 31 => Some(K4_AREA_PX),
            21 | 22 => Some((3100, 3500)),
            _ => None,
        }
    }

    pub fn supports_offline(&self) -> bool {
        matches!(self.0[2], 4 | 31)
    }

    pub fn describe(&self) -> String {
        match self.area_px() {
            None => format!("unknown model {}", self.code()),
            Some((w, h)) => {
                format!("model {}: {w}x{h} px = {}x{} mm", self.code(), w as f64 * PITCH_MM, h as f64 * PITCH_MM)
            }
        }
    }
}

// ------------------------------------------------------------ offline jobs

/// Vendor `get_jiayan`: byte sum, negated when it exceeds 255.
pub fn offline_checksum(data: &[u8]) -> u8 {
    let mut total: i64 = data.iter().map(|&b| b as i64).sum();
    if total > 255 {
        total = -total;
    }
    (total & 0xFF) as u8
}

pub fn offline_job(x: i64, y: i64, width: i64, height: i64, power: u16, depth: u16) -> Result<Vec<u8>> {
    check_power_depth(power, depth)?;
    let chunks = (width * height / 8) / OFFLINE_CHUNK as i64 + 1;
    let p = [
        u16_field(x, "x")?,
        u16_field(y, "y")?,
        u16_field(width, "width")?,
        u16_field(height, "height")?,
        power.to_be_bytes(),
        depth.to_be_bytes(),
        u16_field(chunks, "chunks")?,
    ];
    frame(OP_OFFLINE_JOB, &p.concat())
}

/// Offline raster line: inverted bits (0 = burn) followed by a checksum.
pub fn offline_row(bits: &[u8]) -> Result<Vec<u8>> {
    let mut body: Vec<u8> = bits.iter().map(|b| !b).collect();
    body.push(0);
    let mut out = frame(OP_OFFLINE_ROW, &body)?;
    out.pop();
    let sum = offline_checksum(&out);
    out.push(sum);
    Ok(out)
}

// ----------------------------------------------------------------- decoding

/// A decoded host -> device frame.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Frame {
    pub op: u8,
    pub data: Vec<u8>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Fields {
    None,
    Raster { depth: u16, power: u16, row: u16, bits: Vec<u8> },
    Vector { depth: u16, power: u16, segments: Vec<Segment> },
    Start { x: u16, y: u16 },
    MoveRel { dx: i16, dy: i16 },
    FrameOn { width: u16, height: u16, x: u16, y: u16 },
    Settings(MachineSettings),
}

impl Frame {
    pub fn name(&self) -> String {
        op_name(self.op)
    }

    fn u16_at(&self, i: usize) -> u16 {
        u16::from_be_bytes([self.data[i], self.data[i + 1]])
    }

    pub fn fields(&self) -> Fields {
        let d = &self.data;
        let n = d.len();
        match self.op {
            OP_RASTER_ROW if n >= 9 => Fields::Raster {
                depth: self.u16_at(3),
                power: self.u16_at(5),
                row: self.u16_at(7),
                bits: d[9..].to_vec(),
            },
            OP_VECTOR if n >= 7 => Fields::Vector {
                depth: self.u16_at(3),
                power: self.u16_at(5),
                segments: d[7..]
                    .as_chunks::<8>()
                    .0
                    .iter()
                    .map(|c| {
                        let u = |i: usize| u16::from_be_bytes([c[i], c[i + 1]]);
                        [u(0), u(2), u(4), u(6)]
                    })
                    .collect(),
            },
            OP_START if n >= 7 => Fields::Start { x: self.u16_at(3), y: self.u16_at(5) },
            OP_MOVE_REL if n >= 7 => Fields::MoveRel { dx: self.u16_at(3) as i16, dy: self.u16_at(5) as i16 },
            OP_FRAME_ON if n >= 11 => {
                Fields::FrameOn { width: self.u16_at(3), height: self.u16_at(5), x: self.u16_at(7), y: self.u16_at(9) }
            }
            OP_SETTINGS => MachineSettings::decode(&d[3..]).map_or(Fields::None, Fields::Settings),
            _ => Fields::None,
        }
    }

    pub fn describe(&self) -> String {
        let name = self.name();
        match self.fields() {
            Fields::None => name,
            Fields::Raster { depth, power, row, bits } => {
                let burn: u32 = bits.iter().map(|b| b.count_ones()).sum();
                format!("{name} row={row} power={power} depth={depth} bytes={} burn_px={burn}", bits.len())
            }
            Fields::Vector { depth, power, segments } => {
                let count = |m| segments.iter().filter(|&&s| s == m).count();
                let (on, off) = (count(VECTOR_LASER_ON), count(VECTOR_LASER_OFF));
                format!("{name} power={power} depth={depth} segments={} on={on} off={off}", segments.len() - on - off)
            }
            Fields::Start { x, y } => format!("{name} x={x} y={y}"),
            Fields::MoveRel { dx, dy } => format!("{name} dx={dx} dy={dy}"),
            Fields::FrameOn { width, height, x, y } => {
                format!("{name} width={width} height={height} x={x} y={y}")
            }
            Fields::Settings(s) => format!("{name} {s:?}"),
        }
    }
}

/// Split a host -> device byte stream into frames (every frame carries its length).
pub fn split_frames(stream: &[u8]) -> Result<Vec<Frame>> {
    let mut out = Vec::new();
    let mut i = 0;
    while i < stream.len() {
        if i + 3 > stream.len() {
            return Err(ProtocolError(format!("truncated header at offset {i}")));
        }
        let len = u16::from_be_bytes([stream[i + 1], stream[i + 2]]) as usize;
        if len < 4 || i + len > stream.len() {
            return Err(ProtocolError(format!("bad length {len} at offset {i}")));
        }
        out.push(Frame { op: stream[i], data: stream[i..i + len].to_vec() });
        i += len;
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hex(s: &str) -> Vec<u8> {
        let s: String = s.split_whitespace().collect();
        (0..s.len()).step_by(2).map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap()).collect()
    }

    #[test]
    fn simple_commands() {
        assert_eq!(handshake(), hex("0a000400"));
        assert_eq!(home(), hex("17000400"));
        assert_eq!(reset(), hex("06000400"));
        assert_eq!(stop(), hex("16000400"));
        assert_eq!(pause(), hex("18000400"));
        assert_eq!(resume(), hex("19000400"));
        assert_eq!(frame_off(), hex("21000400"));
        assert_eq!(query_model(), hex("ff000400"));
    }

    #[test]
    fn positioned_commands() {
        assert_eq!(start(0x0102, 0x0304).unwrap(), hex("14 0007 0102 0304"));
        assert_eq!(move_relative(-1, 2).unwrap(), hex("01 0007 ffff 0002"));
        assert_eq!(frame_on(100, 200, 3, 4).unwrap(), hex("20 000b 0064 00c8 0003 0004"));
    }

    #[test]
    fn settings_defaults_match_vendor_ini() {
        let expected = [0x28, 0, 19, 65, 0, 100, 0x03, 0xE8, 8, 1, 1, 0, 200, 0x02, 0x8A, 0x01, 0x90, 0, 10];
        assert_eq!(MachineSettings::default().encode(), expected);
        let fr = &split_frames(&expected).unwrap()[0];
        assert_eq!(fr.fields(), Fields::Settings(MachineSettings::default()));
    }

    #[test]
    fn raster_row_layout() {
        let fr = raster_row(7, &[0x80, 0x01], 1000, 10).unwrap();
        assert_eq!(fr, hex("09 000b 000a 03e8 0007 8001"));
    }

    #[test]
    fn vector_batch_full_length() {
        let segs: Vec<Segment> = (0..80).map(|i| [i, i, i + 1, i + 1]).collect();
        let fr = vector_batch(&segs, 500, 20).unwrap();
        assert_eq!(&fr[..7], &[0x1D, 0x02, 0x87, 0, 20, 0x01, 0xF4]);
        assert_eq!(fr.len(), 647);
        let mut too_many = segs.clone();
        too_many.push([0, 0, 1, 1]);
        assert!(vector_batch(&too_many, 500, 20).is_err());
    }

    #[test]
    fn vector_stream_moves_with_the_laser_off() {
        let (on, off) = (VECTOR_LASER_ON, VECTOR_LASER_OFF);
        // two joined segments, then a separate one
        let paths = [[10, 10, 20, 10], [20, 10, 20, 20], [50, 50, 60, 50]];
        let pass = [on, paths[0], paths[1], off, [20, 20, 50, 50], on, paths[2], off];
        let s = vector_stream(&paths, 1);
        assert_eq!(s, [&[off, [10, 10, 10, 10]][..], &pass, &[[60, 50, 0, 0]]].concat());
        // the second pass starts from where the first ended
        let s2 = vector_stream(&paths, 2);
        assert_eq!(s2, [&[off, [10, 10, 10, 10]][..], &pass, &[[60, 50, 10, 10]], &pass, &[[60, 50, 0, 0]]].concat());
        // a closed loop runs on without a stop
        let square = [[0, 0, 9, 0], [9, 0, 9, 9], [9, 9, 0, 9], [0, 9, 0, 0]];
        assert_eq!(vector_stream(&square, 2).len(), 3 + 8 + 2);
        for n in 0..4 {
            assert_eq!(vector_stream_len(&paths, n), vector_stream(&paths, n).len());
            assert_eq!(vector_stream_len(&square, n), vector_stream(&square, n).len());
        }
        assert!(vector_stream(&[], 2).is_empty() && vector_stream_len(&[], 2) == 0);
        let fr = vector_batch(&s[..1], 500, 20).unwrap();
        assert_eq!(&fr[7..], &[0xC3, 0x50, 0xC3, 0x50, 0xC3, 0x50, 0xC3, 0x50]);
    }

    #[test]
    fn ranges() {
        assert!(raster_row(0, &[0], 1001, 1).is_err());
        assert!(start(-1, 0).is_err());
        assert!(move_relative(40000, 0).is_err());
    }

    #[test]
    fn offline_checksum_and_row() {
        assert_eq!(offline_checksum(&[1, 2, 3]), 6);
        assert_eq!(offline_checksum(&[200, 100]), ((-300i64) & 0xFF) as u8);
        let row = offline_row(&[0xFF, 0x00]).unwrap();
        assert_eq!(&row[..5], &[0x22, 0, 6, 0x00, 0xFF]);
        assert_eq!(row[5], offline_checksum(&row[..5]));
    }

    #[test]
    fn split_roundtrip() {
        let stream = [handshake(), start(5, 6).unwrap(), raster_row(1, &[0x0F], 10, 1).unwrap(), stop()].concat();
        let names: Vec<String> = split_frames(&stream).unwrap().iter().map(Frame::name).collect();
        assert_eq!(names, ["HANDSHAKE", "START", "RASTER_ROW", "STOP"]);
    }
}
