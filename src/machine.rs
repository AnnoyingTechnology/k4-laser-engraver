//! High-level machine control: connect, home, jog, frame, run jobs.
//!
//! Design rules (deliberate, see README "Safety model"):
//!
//! * One owner thread talks to the port; nothing else writes to it.
//! * Every command waits for its ACK; `0x08` triggers a bounded resend.
//! * Pause is host-side: the next row/batch is held back, so the beam is
//!   never left parked by a firmware pause whose ACK semantics are unknown.
//! * Stop sends `0x16` immediately, also while waiting for an ACK, and on
//!   every error path.
//! * The head position is tracked only while known (after HOME); otherwise
//!   the next positioning command homes first.

use std::fmt;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use anyhow::{Result, anyhow, bail};

use crate::plan::{Plan, Rect};
use crate::protocol::{self as P, MachineSettings, Model};
use crate::transport::Port;

/// The job was stopped by the user; STOP has been sent.
#[derive(Debug, Clone, Copy)]
pub struct Aborted;

impl fmt::Display for Aborted {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        f.write_str("stopped by user")
    }
}

impl std::error::Error for Aborted {}

/// Cross-thread job control flags.
#[derive(Debug, Default)]
pub struct Control {
    pub cancel: AtomicBool,
    pub paused: AtomicBool,
}

impl Control {
    pub fn new() -> Arc<Self> {
        Arc::new(Self::default())
    }

    pub fn cancelled(&self) -> bool {
        self.cancel.load(Ordering::SeqCst)
    }

    pub fn reset(&self) {
        self.cancel.store(false, Ordering::SeqCst);
        self.paused.store(false, Ordering::SeqCst);
    }
}

#[derive(Debug, Clone)]
pub struct Progress {
    pub step: usize,
    pub total: usize,
    pub label: String,
    pub elapsed: Duration,
    pub eta: Option<Duration>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dir {
    Tx,
    Rx,
}

pub type TraceFn = Box<dyn FnMut(Dir, &[u8]) + Send>;
pub type LogFn = Box<dyn Fn(&str) + Send>;

pub struct Machine {
    port: Box<dyn Port>,
    pub area_px: (u32, u32),
    pub model: Option<Model>,
    /// Head position in machine px, when known.
    pub head: Option<(u32, u32)>,
    /// Rectangle currently traced by the framing beam.
    pub framing: Option<Rect>,
    pub trace: Option<TraceFn>,
    pub log: LogFn,
    pub ack_timeout: Duration,
    pub motion_timeout: Duration,
    /// One raster row or vector batch, i.e. its burn time.
    pub data_timeout: Duration,
    pub max_resends: u32,
}

impl Machine {
    pub fn new(port: Box<dyn Port>) -> Self {
        Self {
            port,
            area_px: P::K4_AREA_PX,
            model: None,
            head: None,
            framing: None,
            trace: None,
            log: Box::new(|m| eprintln!("{m}")),
            ack_timeout: Duration::from_secs(2),
            motion_timeout: Duration::from_secs(30),
            data_timeout: Duration::from_secs(300),
            max_resends: 10,
        }
    }

    pub fn port_name(&self) -> String {
        self.port.name()
    }

    // ------------------------------------------------------------ low level

    fn send(&mut self, frame: &[u8]) -> Result<()> {
        if let Some(t) = self.trace.as_mut() {
            t(Dir::Tx, frame);
        }
        self.port.write_all(frame)?;
        Ok(())
    }

    fn recv(&mut self, timeout: Duration) -> Result<Vec<u8>> {
        let data = self.port.read(timeout)?;
        if !data.is_empty()
            && let Some(t) = self.trace.as_mut()
        {
            t(Dir::Rx, &data);
        }
        Ok(data)
    }

    /// Wait for ACK/NAK; `None` on timeout.
    fn wait_reply(&mut self, timeout: Duration, control: Option<&Control>) -> Result<Option<u8>> {
        let deadline = Instant::now() + timeout;
        loop {
            if control.is_some_and(Control::cancelled) {
                return Err(Aborted.into());
            }
            let now = Instant::now();
            if now >= deadline {
                return Ok(None);
            }
            let data = self.recv((deadline - now).min(Duration::from_millis(50)))?;
            for b in data {
                if b == P::ACK || b == P::NAK {
                    return Ok(Some(b));
                }
                (self.log)(&format!("unexpected byte from machine: 0x{b:02X}"));
            }
        }
    }

    /// Send `frame` and wait for ACK, resending on NAK. `Ok(false)`: no ACK, tolerated.
    pub fn command(
        &mut self,
        frame: &[u8],
        timeout: Option<Duration>,
        control: Option<&Control>,
        required: bool,
    ) -> Result<bool> {
        let timeout = timeout.unwrap_or(self.ack_timeout);
        let name = P::op_name(frame[0]);
        for _ in 0..=self.max_resends {
            self.send(frame)?;
            match self.wait_reply(timeout, control)? {
                Some(P::ACK) => return Ok(true),
                Some(_) => {
                    (self.log)(&format!("machine requested resend of {name}"));
                    continue;
                }
                None if required => bail!("no ACK for {name} within {} s", timeout.as_secs_f64()),
                None => {
                    (self.log)(&format!("no ACK for {name} (tolerated, as in the vendor app)"));
                    return Ok(false);
                }
            }
        }
        bail!("{name}: {} resend requests", self.max_resends)
    }

    /// Best effort, never fails: abort the job and switch the framing beam off.
    pub fn emergency_stop(&mut self) {
        let mut frames = vec![P::stop()];
        if self.framing.take().is_some() {
            frames.push(P::frame_off());
        }
        for f in frames {
            if let Err(e) = self.send(&f) {
                (self.log)(&format!("stop frame not sent: {e}"));
            }
        }
        self.head = None;
    }

    // ------------------------------------------------------------- session

    /// Handshake, settings, model query, optional home (vendor order).
    pub fn connect(&mut self, settings: Option<&MachineSettings>, home: bool) -> Result<Model> {
        self.port.flush_input()?;
        let mut ok = false;
        for _ in 0..3 {
            if self.command(&P::handshake(), Some(Duration::from_millis(500)), None, false)? {
                ok = true;
                break;
            }
        }
        if !ok {
            bail!("{}: no handshake reply — not a K4, or powered off", self.port.name());
        }
        if let Some(s) = settings {
            self.command(&s.encode(), None, None, false)?;
        }
        let model = self.query_model()?;
        self.model = Some(model);
        if let Some(area) = model.area_px() {
            self.area_px = area;
        }
        if home {
            self.home()?;
        }
        Ok(model)
    }

    /// Identify the machine without moving it or changing its settings.
    pub fn probe(&mut self) -> Result<Model> {
        self.connect(None, false)
    }

    pub fn query_model(&mut self) -> Result<Model> {
        self.port.flush_input()?;
        self.send(&P::query_model())?;
        let mut buf = Vec::new();
        let deadline = Instant::now() + Duration::from_secs(3);
        while buf.len() < 3 && Instant::now() < deadline {
            buf.extend(self.recv(Duration::from_millis(50))?);
        }
        if buf.len() < 3 {
            bail!("model query: got {}", if buf.is_empty() { "nothing".into() } else { hex(&buf) });
        }
        Ok(Model([buf[0], buf[1], buf[2]]))
    }

    pub fn home(&mut self) -> Result<()> {
        self.frame_stop()?;
        self.command(&P::home(), Some(self.motion_timeout), None, false)?;
        self.head = Some((0, 0));
        Ok(())
    }

    pub fn move_to(&mut self, x: u32, y: u32) -> Result<()> {
        self.check_point(x, y)?;
        if self.head.is_none() {
            self.home()?;
        }
        let (hx, hy) = self.head.unwrap();
        let (dx, dy) = (x as i64 - hx as i64, y as i64 - hy as i64);
        if dx != 0 || dy != 0 {
            self.command(&P::move_relative(dx, dy)?, Some(self.motion_timeout), None, true)?;
        }
        self.head = Some((x, y));
        Ok(())
    }

    pub fn jog(&mut self, dx: i64, dy: i64) -> Result<()> {
        if self.head.is_none() {
            self.home()?;
        }
        let (hx, hy) = self.head.unwrap();
        let x = (hx as i64 + dx).clamp(0, self.area_px.0 as i64) as u32;
        let y = (hy as i64 + dy).clamp(0, self.area_px.1 as i64) as u32;
        self.move_to(x, y)
    }

    /// Hidden V2.7 command 0xFD/0xFE; purpose unconfirmed.
    pub fn aux(&mut self, on: bool) -> Result<bool> {
        self.command(&P::aux(on), None, None, false)
    }

    // ------------------------------------------------------------- framing

    /// Trace `rect` with the low-power beam until `frame_stop()`.
    pub fn frame_start(&mut self, rect: Rect) -> Result<()> {
        self.check_point(rect.x, rect.y)?;
        self.check_point(rect.x + rect.w, rect.y + rect.h)?;
        self.frame_stop()?;
        self.move_to(rect.x, rect.y)?;
        self.command(&P::frame_on(rect.w as i64, rect.h as i64, 0, 0)?, Some(self.motion_timeout), None, true)?;
        self.framing = Some(rect);
        Ok(())
    }

    pub fn frame_stop(&mut self) -> Result<()> {
        if self.framing.take().is_some() {
            self.command(&P::frame_off(), None, None, false)?;
        }
        Ok(())
    }

    // ---------------------------------------------------------------- jobs

    /// Execute every operation of `plan`; always ends with STOP.
    pub fn run(&mut self, plan: &Plan, control: &Control, mut progress: impl FnMut(Progress)) -> Result<()> {
        for op in &plan.ops {
            self.check_point(op.x, op.y)?;
            self.check_point(op.x + op.width, op.y + op.height)?;
        }
        let result = self.run_inner(plan, control, &mut progress);
        if result.is_err() {
            self.emergency_stop();
        }
        result
    }

    fn run_inner(&mut self, plan: &Plan, control: &Control, progress: &mut dyn FnMut(Progress)) -> Result<()> {
        let total = plan.data_frames();
        let t0 = Instant::now();
        let (mut done, mut busy) = (0usize, Duration::ZERO);
        self.frame_stop()?;
        let n_ops = plan.ops.len();
        for (index, op) in plan.ops.iter().enumerate() {
            let frames = op.frames()?;
            for npass in 1..=op.passes {
                let label = format!("op {}/{n_ops} “{}” pass {npass}/{}", index + 1, op.name, op.passes);
                (self.log)(&format!("start {label}"));
                self.move_to(op.x, op.y)?;
                self.command(&P::reset(), None, None, false)?;
                self.command(&P::start(op.x as i64, op.y as i64)?, None, None, false)?;
                for frame in &frames {
                    hold_while_paused(control)?;
                    let t1 = Instant::now();
                    self.command(frame, Some(self.data_timeout), Some(control), true)?;
                    done += 1;
                    busy += t1.elapsed();
                    let eta = (done > 0).then(|| busy / done as u32 * (total - done) as u32);
                    progress(Progress { step: done, total, label: label.clone(), elapsed: t0.elapsed(), eta });
                }
                self.send(&P::stop())?;
                self.head = None;
            }
        }
        Ok(())
    }

    fn check_point(&self, x: u32, y: u32) -> Result<()> {
        if x > self.area_px.0 || y > self.area_px.1 {
            return Err(anyhow!(
                "({:.2}, {:.2}) mm is outside the {}x{} mm work area",
                x as f64 * P::PITCH_MM,
                y as f64 * P::PITCH_MM,
                self.area_px.0 as f64 * P::PITCH_MM,
                self.area_px.1 as f64 * P::PITCH_MM
            ));
        }
        Ok(())
    }
}

impl Drop for Machine {
    /// Never leave the framing beam running behind a dropped session.
    fn drop(&mut self) {
        if self.framing.is_some() {
            self.emergency_stop();
        }
    }
}

fn hold_while_paused(control: &Control) -> Result<()> {
    while control.paused.load(Ordering::SeqCst) && !control.cancelled() {
        std::thread::sleep(Duration::from_millis(50));
    }
    if control.cancelled() {
        return Err(Aborted.into());
    }
    Ok(())
}

pub fn hex(data: &[u8]) -> String {
    data.iter().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(" ")
}
