//! Serial transport (raw termios via libc) and a simulated machine.
//!
//! The real port is opened exclusively: `flock` (honoured by lock-aware
//! tools) plus `TIOCEXCL` (any further non-root `open` fails with EBUSY).
//! CH340 bridges have no USB serial number, so `/dev/serial/by-id` names
//! collide between adapters; ports are identified by `/dev/serial/by-path`
//! (physical USB port) instead.

use std::collections::BTreeMap;
use std::ffi::CString;
use std::fs;
use std::io;
use std::os::unix::ffi::OsStrExt;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Condvar, Mutex};
use std::time::{Duration, Instant};

use crate::protocol::{self as P, Frame};

const CH340: (&str, &str) = ("1a86", "7523");

/// Byte pipe to a machine; implemented by the real port and the simulator.
pub trait Port: Send {
    fn name(&self) -> String;
    fn write_all(&mut self, data: &[u8]) -> io::Result<()>;
    /// Available bytes, or an empty vector after `timeout`.
    fn read(&mut self, timeout: Duration) -> io::Result<Vec<u8>>;
    fn flush_input(&mut self) -> io::Result<()>;
}

// ------------------------------------------------------------ port discovery

#[derive(Debug, Clone)]
pub struct PortInfo {
    pub device: PathBuf,
    pub by_path: Option<PathBuf>,
    pub vid: Option<String>,
    pub pid: Option<String>,
    pub product: Option<String>,
    /// sysfs USB topology, e.g. `1-5.1`.
    pub usb_path: Option<String>,
    pub holders: Vec<String>,
}

impl PortInfo {
    pub fn is_ch340(&self) -> bool {
        self.vid.as_deref() == Some(CH340.0) && self.pid.as_deref() == Some(CH340.1)
    }

    pub fn stable_path(&self) -> &Path {
        self.by_path.as_deref().unwrap_or(&self.device)
    }

    pub fn label(&self) -> String {
        let dev = self.device.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or_default();
        let ids = format!("{}:{}", self.vid.as_deref().unwrap_or("?"), self.pid.as_deref().unwrap_or("?"));
        let usb = self.usb_path.as_deref().map(|p| format!("USB {p}")).unwrap_or_else(|| "?".into());
        let held =
            if self.holders.is_empty() { String::new() } else { format!(" — IN USE by {}", self.holders.join(", ")) };
        format!("{dev} ({ids}, {usb}){held}")
    }
}

fn read_trim(path: impl AsRef<Path>) -> Option<String> {
    fs::read_to_string(path).ok().map(|s| s.trim().to_string())
}

/// Processes that currently have `device` open (best effort: own user only).
pub fn port_holders(device: &Path) -> Vec<String> {
    let Ok(target) = fs::canonicalize(device) else { return Vec::new() };
    let me = std::process::id().to_string();
    let mut out = Vec::new();
    let Ok(procs) = fs::read_dir("/proc") else { return out };
    for entry in procs.flatten() {
        let pid = entry.file_name().to_string_lossy().into_owned();
        if !pid.bytes().all(|b| b.is_ascii_digit()) || pid == me {
            continue;
        }
        let Ok(fds) = fs::read_dir(entry.path().join("fd")) else { continue };
        let holds = fds.flatten().any(|fd| fs::read_link(fd.path()).is_ok_and(|l| l == target));
        if holds {
            let comm = read_trim(entry.path().join("comm")).unwrap_or_else(|| "?".into());
            let cmd = fs::read(entry.path().join("cmdline"))
                .map(|b| String::from_utf8_lossy(&b).replace('\0', " ").trim().to_string())
                .unwrap_or_default();
            let cmd: String = cmd.chars().take(80).collect();
            out.push(if cmd.is_empty() { format!("pid {pid} {comm}") } else { format!("pid {pid} {comm} ({cmd})") });
        }
    }
    out
}

/// USB serial ports (ttyUSB*, ttyACM*) with their stable by-path names.
pub fn list_ports() -> Vec<PortInfo> {
    let mut by_path: BTreeMap<PathBuf, PathBuf> = BTreeMap::new();
    if let Ok(entries) = fs::read_dir("/dev/serial/by-path") {
        let mut links: Vec<PathBuf> = entries.flatten().map(|e| e.path()).collect();
        links.sort();
        for link in links {
            if link.to_string_lossy().contains("-usbv2-") {
                continue; // duplicate alias of the same port
            }
            if let Ok(real) = fs::canonicalize(&link) {
                by_path.entry(real).or_insert(link);
            }
        }
    }
    let mut names: Vec<String> = fs::read_dir("/sys/class/tty")
        .map(|d| d.flatten().map(|e| e.file_name().to_string_lossy().into_owned()).collect())
        .unwrap_or_default();
    names.retain(|n| n.starts_with("ttyUSB") || n.starts_with("ttyACM"));
    names.sort();
    names
        .into_iter()
        .map(|name| {
            let device = PathBuf::from(format!("/dev/{name}"));
            let mut info = PortInfo {
                by_path: by_path.get(&device).cloned(),
                holders: port_holders(&device),
                device,
                vid: None,
                pid: None,
                product: None,
                usb_path: None,
            };
            let mut node = fs::canonicalize(format!("/sys/class/tty/{name}/device")).ok();
            while let Some(dir) = node {
                if dir.join("idVendor").exists() {
                    info.vid = read_trim(dir.join("idVendor"));
                    info.pid = read_trim(dir.join("idProduct"));
                    info.product = read_trim(dir.join("product"));
                    info.usb_path = dir.file_name().map(|n| n.to_string_lossy().into_owned());
                    break;
                }
                node = dir.parent().filter(|p| *p != Path::new("/")).map(Path::to_path_buf);
            }
            info
        })
        .collect()
}

// --------------------------------------------------------------- real port

/// Raw 115200-8N1 port, exclusive, non-blocking with poll() timeouts.
pub struct SerialPort {
    path: PathBuf,
    fd: libc::c_int,
}

fn cvt(ret: libc::c_int) -> io::Result<libc::c_int> {
    if ret < 0 { Err(io::Error::last_os_error()) } else { Ok(ret) }
}

impl SerialPort {
    pub fn open(path: &Path, allow_shared: bool) -> io::Result<Self> {
        let holders = port_holders(path);
        if !holders.is_empty() && !allow_shared {
            return Err(io::Error::new(
                io::ErrorKind::ResourceBusy,
                format!("{} is already open by: {}", path.display(), holders.join("; ")),
            ));
        }
        let c_path = CString::new(path.as_os_str().as_bytes())?;
        // SAFETY: plain syscalls on a descriptor we own; closed on every error path.
        unsafe {
            let fd =
                cvt(libc::open(c_path.as_ptr(), libc::O_RDWR | libc::O_NOCTTY | libc::O_NONBLOCK | libc::O_CLOEXEC))?;
            let port = SerialPort { path: path.to_path_buf(), fd };
            if libc::flock(fd, libc::LOCK_EX | libc::LOCK_NB) < 0 {
                return Err(io::Error::new(
                    io::ErrorKind::ResourceBusy,
                    format!("{} is locked by another program", path.display()),
                ));
            }
            cvt(libc::ioctl(fd, libc::TIOCEXCL))?;
            let mut tio: libc::termios = std::mem::zeroed();
            cvt(libc::tcgetattr(fd, &mut tio))?;
            libc::cfmakeraw(&mut tio);
            tio.c_cflag &= !(libc::CSIZE | libc::PARENB | libc::CSTOPB | libc::CRTSCTS);
            tio.c_cflag |= libc::CS8 | libc::CREAD | libc::CLOCAL;
            tio.c_iflag &= !(libc::IXON | libc::IXOFF | libc::IXANY);
            tio.c_cc[libc::VMIN] = 0;
            tio.c_cc[libc::VTIME] = 0;
            cvt(libc::cfsetispeed(&mut tio, libc::B115200))?;
            cvt(libc::cfsetospeed(&mut tio, libc::B115200))?;
            cvt(libc::tcsetattr(fd, libc::TCSANOW, &tio))?;
            cvt(libc::tcflush(fd, libc::TCIOFLUSH))?;
            Ok(port)
        }
    }

    fn poll(&self, events: libc::c_short, timeout: Duration) -> io::Result<bool> {
        let mut pfd = libc::pollfd { fd: self.fd, events, revents: 0 };
        let ms = timeout.as_millis().min(i32::MAX as u128) as libc::c_int;
        // SAFETY: one valid pollfd.
        let n = unsafe { libc::poll(&mut pfd, 1, ms) };
        if n < 0 {
            let err = io::Error::last_os_error();
            return if err.kind() == io::ErrorKind::Interrupted { Ok(false) } else { Err(err) };
        }
        if pfd.revents & (libc::POLLHUP | libc::POLLERR | libc::POLLNVAL) != 0 {
            return Err(io::Error::new(io::ErrorKind::NotConnected, format!("{} disconnected", self.path.display())));
        }
        Ok(n > 0)
    }
}

impl Port for SerialPort {
    fn name(&self) -> String {
        self.path.display().to_string()
    }

    fn write_all(&mut self, mut data: &[u8]) -> io::Result<()> {
        let deadline = Instant::now() + Duration::from_secs(5);
        while !data.is_empty() {
            let now = Instant::now();
            if now >= deadline {
                return Err(io::Error::new(
                    io::ErrorKind::TimedOut,
                    format!("write timeout on {}", self.path.display()),
                ));
            }
            if !self.poll(libc::POLLOUT, deadline - now)? {
                continue;
            }
            // SAFETY: writes from a valid slice.
            let n = unsafe { libc::write(self.fd, data.as_ptr().cast(), data.len()) };
            if n < 0 {
                let err = io::Error::last_os_error();
                if matches!(err.kind(), io::ErrorKind::WouldBlock | io::ErrorKind::Interrupted) {
                    continue;
                }
                return Err(err);
            }
            data = &data[n as usize..];
        }
        Ok(())
    }

    fn read(&mut self, timeout: Duration) -> io::Result<Vec<u8>> {
        if !self.poll(libc::POLLIN, timeout)? {
            return Ok(Vec::new());
        }
        let mut buf = [0u8; 4096];
        // SAFETY: reads into a valid buffer.
        let n = unsafe { libc::read(self.fd, buf.as_mut_ptr().cast(), buf.len()) };
        if n < 0 {
            let err = io::Error::last_os_error();
            if matches!(err.kind(), io::ErrorKind::WouldBlock | io::ErrorKind::Interrupted) {
                return Ok(Vec::new());
            }
            return Err(err);
        }
        if n == 0 {
            return Err(io::Error::new(io::ErrorKind::NotConnected, format!("{} disconnected", self.path.display())));
        }
        Ok(buf[..n as usize].to_vec())
    }

    fn flush_input(&mut self) -> io::Result<()> {
        // SAFETY: valid descriptor.
        cvt(unsafe { libc::tcflush(self.fd, libc::TCIFLUSH) }).map(|_| ())
    }
}

impl Drop for SerialPort {
    fn drop(&mut self) {
        // SAFETY: descriptor owned by self; flock is released by close().
        unsafe {
            libc::ioctl(self.fd, libc::TIOCNXCL);
            libc::close(self.fd);
        }
    }
}

// --------------------------------------------------------- simulated machine

#[derive(Default)]
struct SimState {
    sent: Vec<u8>,
    frames: Vec<Frame>,
    pending: Vec<u8>,
    replies: Vec<u8>,
    ready_at: Option<Instant>,
    data_frames: u64,
}

/// Answers like a K4 and records every frame; used for dry runs and tests.
///
/// Clones share state, so a test can keep a handle while `Machine` owns another.
#[derive(Clone)]
pub struct SimulatedPort {
    pub model: [u8; 3],
    /// Simulated burn time per data frame.
    pub row_time: Duration,
    /// Reply NAK to every n-th data frame (0 = never).
    pub nak_every: u64,
    state: Arc<(Mutex<SimState>, Condvar)>,
}

impl Default for SimulatedPort {
    fn default() -> Self {
        Self {
            model: [3, 1, 4],
            row_time: Duration::ZERO,
            nak_every: 0,
            state: Arc::new((Mutex::new(SimState::default()), Condvar::new())),
        }
    }
}

impl SimulatedPort {
    pub fn frames(&self) -> Vec<Frame> {
        self.state.0.lock().unwrap().frames.clone()
    }

    pub fn sent(&self) -> Vec<u8> {
        self.state.0.lock().unwrap().sent.clone()
    }

    fn answer(&self, st: &mut SimState, op: u8) {
        match op {
            P::OP_MODEL => st.replies.extend_from_slice(&self.model),
            P::OP_STOP => {} // the vendor never waits for one
            P::OP_OFFLINE_JOB => st.replies.extend_from_slice(&P::ERASE_DONE),
            P::OP_RASTER_ROW | P::OP_VECTOR | P::OP_OFFLINE_ROW => {
                st.data_frames += 1;
                let nak = self.nak_every > 0 && st.data_frames.is_multiple_of(self.nak_every);
                st.replies.push(if nak { P::NAK } else { P::ACK });
                st.ready_at = Some(Instant::now() + self.row_time);
            }
            _ => st.replies.push(P::ACK),
        }
    }
}

impl Port for SimulatedPort {
    fn name(&self) -> String {
        "simulator".into()
    }

    fn write_all(&mut self, data: &[u8]) -> io::Result<()> {
        let (lock, cond) = &*self.state;
        let mut st = lock.lock().unwrap();
        st.sent.extend_from_slice(data);
        st.pending.extend_from_slice(data);
        while st.pending.len() >= 3 {
            let len = u16::from_be_bytes([st.pending[1], st.pending[2]]) as usize;
            if len < 4 {
                return Err(io::Error::new(io::ErrorKind::InvalidData, format!("simulator: bad frame length {len}")));
            }
            if st.pending.len() < len {
                break;
            }
            let data: Vec<u8> = st.pending.drain(..len).collect();
            let op = data[0];
            st.frames.push(Frame { op, data });
            self.answer(&mut st, op);
        }
        cond.notify_all();
        Ok(())
    }

    fn read(&mut self, timeout: Duration) -> io::Result<Vec<u8>> {
        let deadline = Instant::now() + timeout;
        let (lock, cond) = &*self.state;
        let mut st = lock.lock().unwrap();
        loop {
            let now = Instant::now();
            let ready = st.ready_at.is_none_or(|t| now >= t);
            if !st.replies.is_empty() && ready {
                return Ok(std::mem::take(&mut st.replies));
            }
            if now >= deadline {
                return Ok(Vec::new());
            }
            let mut wait = deadline - now;
            if let (false, Some(t)) = (st.replies.is_empty(), st.ready_at) {
                wait = wait.min(t.saturating_duration_since(now));
            }
            st = cond.wait_timeout(st, wait).unwrap().0;
        }
    }

    fn flush_input(&mut self) -> io::Result<()> {
        self.state.0.lock().unwrap().replies.clear();
        Ok(())
    }
}
