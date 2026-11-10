//! Native GUI (egui/eframe).
//!
//! One window: a top bar to add files and connect the engraver, the bed
//! (always fitted and centred), the item list and the editor of the selected
//! item on the right, and STOP / Frame / Start at the bottom.
//!
//! The machine lives on a worker thread that owns the port; the UI sends it
//! commands and reads events. STOP and Pause go through shared atomics, so
//! they act within 50 ms even while the worker waits for an ACK.

use std::collections::{BTreeMap, HashMap};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::sync::mpsc::{self, Receiver, Sender};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, anyhow, bail};
use eframe::egui::{self, Align2, Color32, Key, KeyboardShortcut, Modifiers, Pos2, RichText, Sense, Stroke, Vec2};

use crate::config::{self, Config};
use crate::geom::Side;
use crate::job::{self, GeometryCache, Job, Operation};
use crate::machine::{Aborted, Control, Machine, Progress};
use crate::plan::{Geometry, OpPlan, Plan, Rect};
use crate::protocol::{self as P, MachineSettings, Model};
use crate::sources::{self, Align, DxfSrc, ImageProcess, ImageSrc, Mode, ShapeKind, ShapeSrc, Source, SvgSrc};
use crate::timing::{self, Estimate, Timing};
use crate::transport::{self, Port, PortInfo, SerialPort, SimulatedPort};

pub fn run(job_path: Option<PathBuf>) -> Result<()> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("k4laser")
            .with_app_id("k4laser")
            .with_inner_size([1360.0, 880.0])
            .with_min_inner_size([960.0, 620.0])
            .with_drag_and_drop(true),
        ..Default::default()
    };
    eframe::run_native("k4laser", options, Box::new(move |cc| Ok(Box::new(App::new(cc, job_path)))))
        .map_err(|e| anyhow!("GUI: {e}"))
}

const RED: Color32 = Color32::from_rgb(200, 35, 35);
const GREEN: Color32 = Color32::from_rgb(40, 140, 70);
const ORANGE: Color32 = Color32::from_rgb(225, 125, 30);
const BLUE: Color32 = Color32::from_rgb(40, 120, 230);
const ERROR: Color32 = Color32::from_rgb(215, 55, 50);
const BED: Color32 = Color32::from_rgb(246, 242, 234);
const LINE: Color32 = Color32::from_rgb(200, 40, 40);
const BEAM: Color32 = Color32::from_rgb(235, 45, 35);

const IMAGE_EXTS: &[&str] = &["png", "jpg", "jpeg", "bmp", "gif", "webp", "tif", "tiff"];
const SVG_EXTS: &[&str] = &["svg", "svgz"];
const DXF_EXTS: &[&str] = &["dxf"];

const DEPTH_HINT: &str = "Dwell per dot: the vendor's \"carving depth\", 0-100 (unit unknown). Higher burns darker \
                          and deeper. Calibrate with Add ▾ → Test grid.";
const OUTLINE_HINT: &str = "Concentric copies of the path, like the walls of a 3D print. They widen the cut so that \
                            a part comes out cleanly.";

// ------------------------------------------------------------------ worker

#[derive(Debug, Clone, PartialEq)]
enum PortChoice {
    Serial(PathBuf),
    Simulator,
}

impl PortChoice {
    fn label(&self) -> String {
        match self {
            PortChoice::Serial(p) => p.display().to_string(),
            PortChoice::Simulator => "the simulator".into(),
        }
    }
}

enum Cmd {
    Connect(PortChoice, MachineSettings),
    Disconnect,
    Home,
    Jog(i64, i64),
    Frame(Rect),
    FrameStop,
    Stop,
    Run(Arc<Plan>, Arc<Control>),
    Aux(bool),
}

enum Event {
    Log(String),
    Connected { model: Model, port: String },
    Disconnected,
    State { head: Option<(u32, u32)>, framing: Option<Rect> },
    Idle,
    Progress(Progress),
    JobDone(Result<(), String>),
    Timing(Box<Timing>),
}

#[derive(Clone)]
struct Out {
    tx: Sender<Event>,
    ctx: egui::Context,
}

impl Out {
    fn send(&self, e: Event) {
        let _ = self.tx.send(e);
        self.ctx.request_repaint();
    }
}

fn is_disconnect(e: &anyhow::Error) -> bool {
    e.chain().any(|c| {
        c.downcast_ref::<std::io::Error>().is_some_and(|io| {
            matches!(io.kind(), std::io::ErrorKind::NotConnected | std::io::ErrorKind::BrokenPipe)
                || matches!(io.raw_os_error(), Some(libc::ENODEV) | Some(libc::EIO) | Some(libc::ENXIO))
        })
    })
}

fn worker(rx: Receiver<Cmd>, out: Out) {
    let mut machine: Option<Machine> = None;
    for cmd in rx {
        if let Err(e) = handle(&mut machine, cmd, &out) {
            out.send(Event::Log(format!("error: {e:#}")));
            if is_disconnect(&e) {
                machine = None;
                out.send(Event::Disconnected);
            }
        }
        if let Some(m) = &machine {
            out.send(Event::State { head: m.head, framing: m.framing });
        }
        out.send(Event::Idle);
    }
    // UI gone: dropping the machine switches a running frame off
}

fn handle(machine: &mut Option<Machine>, cmd: Cmd, out: &Out) -> Result<()> {
    match cmd {
        Cmd::Connect(choice, settings) => {
            *machine = None;
            let port: Box<dyn Port> = match &choice {
                PortChoice::Serial(p) => Box::new(SerialPort::open(p, false).with_context(|| p.display().to_string())?),
                PortChoice::Simulator => {
                    let mut sim = SimulatedPort::default();
                    sim.row_time = Duration::from_millis(4);
                    Box::new(sim)
                }
            };
            let mut m = Machine::new(port);
            if matches!(choice, PortChoice::Serial(_)) {
                m.timing = Some(timing::load());
            }
            let o = out.clone();
            m.log = Box::new(move |s| o.send(Event::Log(s.to_string())));
            // no homing: connecting never moves the head
            let model = m.connect(Some(&settings), false)?;
            out.send(Event::Connected { model, port: m.port_name() });
            *machine = Some(m);
        }
        Cmd::Disconnect => {
            if let Some(mut m) = machine.take() {
                m.frame_stop()?;
            }
            out.send(Event::Disconnected);
        }
        cmd => {
            let Some(m) = machine.as_mut() else { bail!("not connected") };
            match cmd {
                Cmd::Home => m.home()?,
                Cmd::Jog(dx, dy) => m.jog(dx, dy)?,
                Cmd::Frame(r) => m.frame_start(r)?,
                Cmd::FrameStop => m.frame_stop()?,
                Cmd::Stop => {
                    m.emergency_stop();
                    out.send(Event::Log("STOP sent".into()));
                }
                Cmd::Aux(on) => {
                    let ack = m.aux(on)?;
                    out.send(Event::Log(format!(
                        "aux {}: {}",
                        if on { "0xFD" } else { "0xFE" },
                        if ack { "ACK" } else { "no ACK" }
                    )));
                }
                Cmd::Run(plan, control) => {
                    let o = out.clone();
                    let res = m.run(&plan, &control, |p| o.send(Event::Progress(p)));
                    if let Some(t) = &m.timing {
                        if let Err(e) = timing::save(t) {
                            out.send(Event::Log(format!("timing not saved: {e:#}")));
                        }
                        out.send(Event::Timing(Box::new(t.clone())));
                    }
                    let msg = match &res {
                        Ok(()) => Ok(()),
                        Err(e) if e.is::<Aborted>() => Err("stopped — STOP sent".to_string()),
                        Err(e) => Err(format!("{e:#}")),
                    };
                    out.send(Event::JobDone(msg));
                    if let Err(e) = res
                        && is_disconnect(&e)
                    {
                        return Err(e);
                    }
                }
                Cmd::Connect(..) | Cmd::Disconnect => unreachable!(),
            }
        }
    }
    Ok(())
}

// ------------------------------------------------------------------- model

/// Compiled state of one item.
#[derive(Default)]
struct ItemView {
    /// Artwork box on the bed, mm: x, y, width, height.
    frame: Option<[f64; 4]>,
    plans: Vec<OpPlan>,
    error: Option<String>,
    notes: Vec<String>,
    duration: Option<Estimate>,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum FrameScope {
    Job,
    Item(usize),
}

/// Pending placement: once item `op` is compiled, move it so that the point at
/// `frac` of its box lands on `at` (mm). `fit` first shrinks it into the bed.
struct Fix {
    op: usize,
    at: (f64, f64),
    frac: (f64, f64),
    fit: bool,
}

enum Gesture {
    Move {
        op: usize,
        start: (f64, f64),
        frame: [f64; 4],
        grab: (f64, f64),
    },
    /// `corner` is the dragged corner as a fraction of the box; the opposite one stays put.
    Resize {
        op: usize,
        frame: [f64; 4],
        corner: (f64, f64),
        scale: (f64, f64),
    },
}

enum Pending {
    New,
    Open(PathBuf),
}

/// Laser settings given to new items: the last ones used.
#[derive(Clone, Copy)]
struct Laser {
    power: f64,
    min_power: f64,
    depth: u16,
    passes: u32,
}

enum Action {
    AddFiles,
    AddSource(Source),
    AddTestGrid,
    ReplaceFile(usize),
    New,
    Open,
    Save,
    SaveAs,
    Duplicate(usize),
    Delete(usize),
    Reorder(usize, bool),
    Undo,
    Redo,
    Connect(PortChoice),
    Rescan,
    Frame(FrameScope),
    FrameStop,
    Stop,
    Center(usize),
    Rotate(usize, u16),
    Mirror(usize),
    Resize(usize, f64, f64, (f64, f64)),
    Machine(Cmd),
}

/// Bed mm ↔ screen.
#[derive(Clone, Copy)]
struct Tf {
    origin: Pos2,
    z: f64,
}

impl Tf {
    fn pos(&self, x: f64, y: f64) -> Pos2 {
        Pos2::new(self.origin.x + (x * self.z) as f32, self.origin.y + (y * self.z) as f32)
    }
    fn rect(&self, [x, y, w, h]: [f64; 4]) -> egui::Rect {
        egui::Rect::from_min_max(self.pos(x, y), self.pos(x + w, y + h))
    }
    fn mm(&self, p: Pos2) -> (f64, f64) {
        ((p.x - self.origin.x) as f64 / self.z, (p.y - self.origin.y) as f64 / self.z)
    }
}

fn snap(v: f64, step: f64) -> f64 {
    ((v / step).round() * step * 1e6).round() / 1e6
}

/// Box of a resize in progress.
fn resized([x, y, w, h]: [f64; 4], (cx, cy): (f64, f64), (sx, sy): (f64, f64)) -> [f64; 4] {
    let (ax, ay) = (x + (1.0 - cx) * w, y + (1.0 - cy) * h);
    let (nw, nh) = (w * sx, h * sy);
    [if cx > 0.5 { ax } else { ax - nw }, if cy > 0.5 { ay } else { ay - nh }, nw, nh]
}

/// Raster preview colour: darker with more power.
fn burn_tint(power: u16, faded: bool) -> Color32 {
    let a = 40.0 + 215.0 * power.min(1000) as f32 / 1000.0;
    Color32::from_rgba_unmultiplied(46, 30, 18, if faded { a * 0.2 } else { a } as u8)
}

// --------------------------------------------------------------------- app

struct App {
    job: Job,
    job_path: Option<PathBuf>,
    saved: Job,
    selected: Option<usize>,
    cache: GeometryCache,
    compiled: Option<Job>,
    views: Vec<ItemView>,
    plan: Arc<Plan>,
    plan_fp: u64,
    first_error: Option<String>,
    timing: Timing,
    estimate: Option<Estimate>,
    last_change: Instant,
    textures: HashMap<u64, egui::TextureHandle>,
    fixes: Vec<Fix>,
    laser: Laser,
    lock_ratio: bool,
    act: Vec<Action>,
    undo: Vec<Job>,
    redo: Vec<Job>,
    committed: Job,
    last_dir: Option<PathBuf>,

    cfg: Config,
    ports: Vec<PortInfo>,
    ports_at: Option<Instant>,
    port_seen: bool,
    choice: Option<PortChoice>,
    tx: Option<Sender<Cmd>>,
    rx: Receiver<Event>,
    worker: Option<JoinHandle<()>>,
    connected: Option<(Model, String)>,
    pending: usize,
    head: Option<(u32, u32)>,
    framing: Option<Rect>,
    frame_req: Option<(Rect, Option<u64>)>,
    frame_scope: FrameScope,
    framed_fp: Option<u64>,
    running: bool,
    control: Arc<Control>,
    progress: Option<Progress>,
    confirm: bool,
    discard: Option<Pending>,
    jog_mm: f64,

    status: Option<(String, bool)>,
    log: Vec<String>,
    show_log: bool,
    gesture: Option<Gesture>,
    title: String,
    screenshot: Option<(PathBuf, u32)>,
}

impl App {
    fn new(cc: &eframe::CreationContext, job_path: Option<PathBuf>) -> Self {
        install_fallback_font(&cc.egui_ctx);
        let (tx, worker_rx) = mpsc::channel();
        let (worker_tx, rx) = mpsc::channel();
        let out = Out { tx: worker_tx, ctx: cc.egui_ctx.clone() };
        let worker = std::thread::Builder::new()
            .name("k4laser-machine".into())
            .spawn(move || worker(worker_rx, out))
            .expect("spawn machine thread");
        let mut log = Vec::new();
        let cfg = config::load().unwrap_or_else(|e| {
            log.push(format!("config ignored: {e:#}"));
            Config::default()
        });
        let job = Job { base_dir: std::env::current_dir().unwrap_or_default(), ..Default::default() };
        let mut app = App {
            saved: job.clone(),
            committed: job.clone(),
            job,
            job_path: None,
            selected: None,
            cache: GeometryCache::default(),
            compiled: None,
            views: Vec::new(),
            plan: Arc::new(Plan::default()),
            plan_fp: 0,
            first_error: None,
            timing: timing::load(),
            estimate: None,
            last_change: Instant::now(),
            textures: HashMap::new(),
            fixes: Vec::new(),
            laser: Laser { power: 50.0, min_power: 0.0, depth: 10, passes: 1 },
            lock_ratio: true,
            act: Vec::new(),
            undo: Vec::new(),
            redo: Vec::new(),
            last_dir: None,
            cfg,
            ports: Vec::new(),
            ports_at: None,
            port_seen: false,
            choice: None,
            tx: Some(tx),
            rx,
            worker: Some(worker),
            connected: None,
            pending: 0,
            head: None,
            framing: None,
            frame_req: None,
            frame_scope: FrameScope::Job,
            framed_fp: None,
            running: false,
            control: Control::new(),
            progress: None,
            confirm: false,
            discard: None,
            jog_mm: 1.0,
            status: None,
            log,
            show_log: false,
            gesture: None,
            title: String::new(),
            screenshot: std::env::var_os("K4LASER_SCREENSHOT").map(|p| (PathBuf::from(p), 0)),
        };
        if let Some(p) = job_path {
            app.open(&p);
        }
        app
    }

    fn area_px(&self) -> (u32, u32) {
        self.connected.as_ref().and_then(|(m, _)| m.area_px()).unwrap_or(P::K4_AREA_PX)
    }

    fn area_mm(&self) -> (f64, f64) {
        let (w, h) = self.area_px();
        (w as f64 * P::PITCH_MM, h as f64 * P::PITCH_MM)
    }

    fn logln(&mut self, s: impl Into<String>) {
        self.log.push(s.into());
        if self.log.len() > 2000 {
            self.log.drain(..500);
        }
    }

    fn info(&mut self, s: impl Into<String>) {
        let s = s.into();
        self.logln(s.clone());
        self.status = Some((s, false));
    }

    fn fail(&mut self, s: impl Into<String>) {
        let s = s.into();
        self.logln(format!("error: {s}"));
        self.status = Some((s, true));
    }

    fn send(&mut self, cmd: Cmd) {
        if let Some(tx) = &self.tx {
            self.pending += 1;
            let _ = tx.send(cmd);
        }
    }

    fn dirty(&self) -> bool {
        self.job.ops != self.saved.ops || self.job.variables != self.saved.variables
    }

    fn dialog(&self) -> rfd::FileDialog {
        let d = rfd::FileDialog::new();
        match &self.last_dir {
            Some(dir) => d.set_directory(dir),
            None => d,
        }
    }

    fn remember_dir(&mut self, p: &Path) {
        self.last_dir = p.parent().map(Path::to_path_buf);
    }

    // ------------------------------------------------------------- files

    fn open(&mut self, path: &Path) {
        match Job::load(path, &BTreeMap::new()) {
            Ok(job) => {
                self.saved = job.clone();
                self.committed = job.clone();
                self.job = job;
                self.undo.clear();
                self.redo.clear();
                self.job_path = Some(path.to_path_buf());
                self.selected = None;
                self.frame_scope = FrameScope::Job;
                self.remember_dir(path);
                self.info(format!("Opened {}", path.display()));
            }
            Err(e) => self.fail(format!("{e:#}")),
        }
    }

    fn new_job(&mut self) {
        let job = Job { base_dir: self.job.base_dir.clone(), ..Default::default() };
        self.saved = job.clone();
        self.committed = job.clone();
        self.job = job;
        self.undo.clear();
        self.redo.clear();
        self.job_path = None;
        self.selected = None;
        self.frame_scope = FrameScope::Job;
    }

    fn request(&mut self, p: Pending) {
        if self.dirty() {
            self.discard = Some(p);
        } else {
            self.proceed(p);
        }
    }

    fn proceed(&mut self, p: Pending) {
        match p {
            Pending::New => self.new_job(),
            Pending::Open(path) => self.open(&path),
        }
    }

    /// Save to `path`, or to the current file, or ask.
    fn save(&mut self, path: Option<PathBuf>) {
        let path = match path.or_else(|| self.job_path.clone()) {
            Some(p) => p,
            None => {
                match self.dialog().add_filter("k4laser project", &["toml"]).set_file_name("job.toml").save_file() {
                    Some(p) => p,
                    None => return,
                }
            }
        };
        let path = std::path::absolute(&path).unwrap_or(path);
        let dir = path.parent().map(Path::to_path_buf).unwrap_or_default();
        // files next to the project are stored relative to it, others absolute
        let old_base = self.job.base_dir.clone();
        for op in &mut self.job.ops {
            let p = match &mut op.source {
                Source::Image(s) => &mut s.path,
                Source::Svg(s) => &mut s.path,
                Source::Dxf(s) => &mut s.path,
                _ => continue,
            };
            if p.is_empty() || p.contains("${") {
                continue;
            }
            let abs = sources::resolve(p, &old_base);
            *p = match abs.strip_prefix(&dir) {
                Ok(rel) => rel.display().to_string(),
                Err(_) => abs.display().to_string(),
            };
        }
        self.job.base_dir = dir;
        match self.job.save(&path) {
            Ok(()) => {
                self.saved = self.job.clone();
                self.committed = self.job.clone();
                self.job_path = Some(path.clone());
                self.remember_dir(&path);
                self.info(format!("Saved {}", path.display()));
            }
            Err(e) => self.fail(format!("{}: {e:#}", path.display())),
        }
    }

    fn add_files_dialog(&mut self) {
        let all: Vec<&str> = IMAGE_EXTS.iter().chain(SVG_EXTS).chain(DXF_EXTS).copied().collect();
        let Some(paths) = self
            .dialog()
            .add_filter("Images, SVG, DXF", &all)
            .add_filter("Images", IMAGE_EXTS)
            .add_filter("SVG", SVG_EXTS)
            .add_filter("DXF", DXF_EXTS)
            .pick_files()
        else {
            return;
        };
        for p in paths {
            self.add_path(&p);
        }
    }

    fn add_path(&mut self, path: &Path) {
        let abs = std::path::absolute(path).unwrap_or_else(|_| path.to_path_buf());
        self.remember_dir(&abs);
        let ext = abs.extension().map(|e| e.to_string_lossy().to_lowercase()).unwrap_or_default();
        let p = abs.display().to_string();
        let src = if IMAGE_EXTS.contains(&ext.as_str()) {
            let (w, h) = match sources::image_size(&abs) {
                Ok(s) => s,
                Err(e) => return self.fail(format!("{}: {e:#}", abs.display())),
            };
            // a first size that fits a 50 mm box; the user resizes from there
            let f = 50.0 / w.max(h).max(1) as f64;
            let r = |v: f64| (v * 100.0).round() / 100.0;
            Source::Image(ImageSrc {
                path: p,
                width_mm: r(w as f64 * f),
                height_mm: r(h as f64 * f),
                ..Default::default()
            })
        } else if SVG_EXTS.contains(&ext.as_str()) {
            Source::Svg(SvgSrc { path: p, ..Default::default() })
        } else if DXF_EXTS.contains(&ext.as_str()) {
            Source::Dxf(DxfSrc { path: p, ..Default::default() })
        } else if ext == "toml" {
            return self.request(Pending::Open(abs));
        } else {
            return self.fail(format!("{}: not an image, SVG or DXF file", abs.display()));
        };
        self.add_op(Operation::new(src));
    }

    /// Append with the last laser settings, centred and shrunk into the bed.
    fn add_op(&mut self, mut op: Operation) {
        op.power = self.laser.power;
        op.depth = self.laser.depth;
        op.passes = self.laser.passes;
        if op.is_grayscale() {
            op.min_power = self.laser.min_power.min(op.power);
        }
        self.job.ops.push(op);
        let i = self.job.ops.len() - 1;
        self.selected = Some(i);
        let (aw, ah) = self.area_mm();
        self.fixes.push(Fix { op: i, at: (aw / 2.0, ah / 2.0), frac: (0.5, 0.5), fit: true });
    }

    fn replace_file(&mut self, i: usize) {
        let (name, exts) = match self.job.ops.get(i).map(|op| &op.source) {
            Some(Source::Image(_)) => ("Images", IMAGE_EXTS),
            Some(Source::Svg(_)) => ("SVG", SVG_EXTS),
            Some(Source::Dxf(_)) => ("DXF", DXF_EXTS),
            _ => return,
        };
        let Some(p) = self.dialog().add_filter(name, exts).pick_file() else { return };
        let abs = std::path::absolute(&p).unwrap_or(p);
        self.remember_dir(&abs);
        let size = sources::image_size(&abs).ok();
        match &mut self.job.ops[i].source {
            Source::Image(s) => {
                s.path = abs.display().to_string();
                // keep the width, follow the new aspect ratio
                if let Some((w, h)) = size.filter(|&(w, h)| w > 0 && h > 0) {
                    let r = |v: f64| (v * 100.0).round() / 100.0;
                    if s.width_mm > 0.0 {
                        s.height_mm = r(s.width_mm * h as f64 / w as f64);
                    } else if s.height_mm > 0.0 {
                        s.width_mm = r(s.height_mm * w as f64 / h as f64);
                    }
                }
            }
            Source::Svg(s) => s.path = abs.display().to_string(),
            Source::Dxf(d) => d.path = abs.display().to_string(),
            _ => {}
        }
    }

    // ----------------------------------------------------------- compile

    fn recompile(&mut self) {
        if self.compiled.as_ref() == Some(&self.job) {
            return;
        }
        let area = self.area_px();
        let (aw, ah) = self.area_mm();
        let mut views = Vec::with_capacity(self.job.ops.len());
        let mut plan = Plan::default();
        let mut first_error = None;
        for op in &self.job.ops {
            let mut v = ItemView::default();
            match self.cache.get(op, &self.job.base_dir, &self.job.variables) {
                Err(e) => v.error = Some(format!("{e:#}")),
                Ok(prep) => {
                    v.notes = prep.notes.clone();
                    v.frame = prep.frame.map(|[x, y, w, h]| [op.x_mm + x, op.y_mm + y, w, h]);
                    match job::compile_op(op, &prep) {
                        Err(e) => v.error = Some(format!("{e:#}")),
                        Ok(plans) => {
                            if plans.is_empty() {
                                v.notes.push("nothing to burn".into());
                            }
                            if plans.iter().any(|p| job::check_area(p, area).is_err()) {
                                v.error = Some(format!("extends past the {aw:.0} × {ah:.0} mm bed"));
                            }
                            v.plans = plans;
                        }
                    }
                }
            }
            if op.enabled {
                match &v.error {
                    Some(e) => {
                        let label = op.label();
                        first_error.get_or_insert_with(|| {
                            if e.starts_with(&label) { e.clone() } else { format!("{label}: {e}") }
                        });
                    }
                    None => plan.ops.extend(v.plans.iter().cloned()),
                }
            }
            views.push(v);
        }
        self.plan_fp = plan.fingerprint();
        self.plan = Arc::new(plan);
        self.first_error = first_error;
        self.views = views;
        self.compiled = Some(self.job.clone());
        self.last_change = Instant::now();
        self.refresh_estimates();
    }

    /// Durations from the timing model learned on the machine.
    fn refresh_estimates(&mut self) {
        for v in &mut self.views {
            v.duration = (!v.plans.is_empty()).then(|| self.timing.estimate_ops(&v.plans));
        }
        self.estimate = (!self.plan.ops.is_empty()).then(|| self.timing.estimate(&self.plan));
    }

    /// Recompile, then apply pending placements (they need the compiled size).
    fn settle(&mut self) {
        self.recompile();
        for fix in std::mem::take(&mut self.fixes) {
            if fix.op >= self.job.ops.len() {
                continue;
            }
            if fix.fit
                && let Some([_, _, w, h]) = self.views[fix.op].frame
            {
                let (aw, ah) = self.area_mm();
                let f = (aw * 0.9 / w).min(ah * 0.9 / h);
                if f < 1.0 && self.scale_op(fix.op, f, f, (w, h)) {
                    self.recompile();
                }
            }
            if let Some([x, y, w, h]) = self.views[fix.op].frame {
                let op = &mut self.job.ops[fix.op];
                op.x_mm = snap(op.x_mm + fix.at.0 - (x + fix.frac.0 * w), P::PITCH_MM);
                op.y_mm = snap(op.y_mm + fix.at.1 - (y + fix.frac.1 * h), P::PITCH_MM);
                self.recompile();
            }
        }
    }

    /// Scale item `i` by (fx, fy) on the bed, given its current box size.
    fn scale_op(&mut self, i: usize, fx: f64, fy: f64, (w, h): (f64, f64)) -> bool {
        let base = self.job.base_dir.clone();
        let op = &mut self.job.ops[i];
        // the source has its own axes; a quarter turn swaps them
        let turned = op.rotate % 180 == 90;
        let (sfx, sfy, size) = if turned { (fy, fx, (h, w)) } else { (fx, fy, (w, h)) };
        match op.source.scale(sfx, sfy, size, &base) {
            Ok(()) => true,
            Err(e) => {
                self.fail(format!("resize: {e:#}"));
                false
            }
        }
    }

    fn resize(&mut self, i: usize, fx: f64, fy: f64, frac: (f64, f64)) {
        let Some([x, y, w, h]) = self.views.get(i).and_then(|v| v.frame) else { return };
        if !(fx.is_finite() && fy.is_finite() && fx > 0.0 && fy > 0.0) {
            return;
        }
        if self.scale_op(i, fx, fy, (w, h)) {
            self.fixes.push(Fix { op: i, at: (x + frac.0 * w, y + frac.1 * h), frac, fit: false });
        }
    }

    // ------------------------------------------------------------ history

    fn undo_tick(&mut self, ctx: &egui::Context) {
        if self.job == self.committed {
            return;
        }
        let settled = self.gesture.is_none()
            && self.last_change.elapsed() > Duration::from_millis(500)
            && !ctx.input(|i| i.pointer.any_down());
        if settled {
            self.undo.push(std::mem::replace(&mut self.committed, self.job.clone()));
            if self.undo.len() > 200 {
                self.undo.remove(0);
            }
            self.redo.clear();
        } else {
            ctx.request_repaint_after(Duration::from_millis(550));
        }
    }

    fn undo(&mut self) {
        if self.job != self.committed {
            self.undo.push(std::mem::replace(&mut self.committed, self.job.clone()));
        }
        if let Some(prev) = self.undo.pop() {
            self.redo.push(std::mem::replace(&mut self.committed, prev));
            self.job = self.committed.clone();
            self.after_history();
        }
    }

    fn redo(&mut self) {
        if self.job != self.committed {
            return;
        }
        if let Some(next) = self.redo.pop() {
            self.undo.push(std::mem::replace(&mut self.committed, next));
            self.job = self.committed.clone();
            self.after_history();
        }
    }

    fn after_history(&mut self) {
        self.selected = self.selected.filter(|&i| i < self.job.ops.len());
        self.frame_scope = FrameScope::Job;
        self.gesture = None;
    }

    // ------------------------------------------------------------ machine

    fn frame_target(&self, scope: FrameScope) -> Option<(Rect, Option<u64>)> {
        match scope {
            FrameScope::Job => {
                if self.first_error.is_some() {
                    return None;
                }
                self.plan.bbox().map(|r| (r, Some(self.plan_fp)))
            }
            FrameScope::Item(i) => {
                let v = self.views.get(i).filter(|v| v.error.is_none())?;
                Plan { ops: v.plans.clone() }.bbox().map(|r| (r, None))
            }
        }
    }

    fn start_frame(&mut self, scope: FrameScope) {
        if let Some((rect, fp)) = self.frame_target(scope) {
            self.frame_scope = scope;
            self.frame_req = Some((rect, fp));
            self.send(Cmd::Frame(rect));
        }
    }

    fn framing_active(&self) -> bool {
        self.framing.is_some() || self.frame_req.is_some()
    }

    fn stop(&mut self) {
        if self.running {
            self.control.cancel.store(true, Ordering::SeqCst);
            self.logln("STOP requested");
        } else if self.framing_active() {
            self.frame_req = None;
            self.send(Cmd::FrameStop);
        } else if self.connected.is_some() {
            self.send(Cmd::Stop);
        }
    }

    /// Ok, or why Start is not available.
    fn can_start(&self) -> Result<(), String> {
        if self.running {
            Err("Engraving…".into())
        } else if self.connected.is_none() {
            Err("Connect the engraver (top right)".into())
        } else if self.job.ops.is_empty() {
            Err("Add something to engrave".into())
        } else if let Some(e) = &self.first_error {
            Err(e.clone())
        } else if self.plan.ops.is_empty() {
            Err("Nothing to burn: every item is disabled or empty".into())
        } else if self.pending > 0 {
            Err("Machine busy…".into())
        } else {
            Ok(())
        }
    }

    fn start_job(&mut self) {
        if self.can_start().is_err() {
            return;
        }
        self.confirm = false;
        self.control = Control::new();
        self.running = true;
        self.progress = None;
        self.status = None;
        self.frame_req = None;
        self.gesture = None;
        let plan = self.plan.clone();
        self.logln(format!("job started: {} part(s)", plan.ops.len()));
        // the machine switches framing off before burning
        self.send(Cmd::Run(plan, self.control.clone()));
    }

    fn connect(&mut self, choice: PortChoice) {
        self.logln(format!("connecting to {}…", choice.label()));
        self.choice = Some(choice.clone());
        self.send(Cmd::Connect(choice, self.cfg.settings));
    }

    /// Rescan ports while disconnected; connect when the remembered port appears.
    fn poll_ports(&mut self) {
        if self.connected.is_some() || self.ports_at.is_some_and(|t| t.elapsed() < Duration::from_secs(2)) {
            return;
        }
        self.ports_at = Some(Instant::now());
        self.ports = transport::list_ports();
        self.ports.sort_by_key(|p| !p.is_ch340());
        let present =
            self.cfg.port.as_ref().is_some_and(|want| {
                self.ports.iter().any(|p| p.stable_path() == want.as_path() && p.holders.is_empty())
            });
        // only the port the user picked before, only on (re)appearance, never in self-tests
        if present && !self.port_seen && self.pending == 0 && self.screenshot.is_none() {
            let port = self.cfg.port.clone().unwrap();
            self.connect(PortChoice::Serial(port));
        }
        self.port_seen = present;
    }

    fn drain_events(&mut self) {
        while let Ok(ev) = self.rx.try_recv() {
            match ev {
                Event::Log(s) => {
                    if let Some(e) = s.strip_prefix("error: ") {
                        self.status = Some((e.to_string(), true));
                    }
                    self.logln(s);
                }
                Event::Connected { model, port } => {
                    self.logln(format!("connected to {port}: {}", model.describe()));
                    if let Some(PortChoice::Serial(p)) = &self.choice
                        && self.cfg.port.as_ref() != Some(p)
                    {
                        self.cfg.port = Some(p.clone());
                        if let Err(e) = config::save(&self.cfg) {
                            self.logln(format!("config not saved: {e:#}"));
                        }
                    }
                    self.connected = Some((model, port));
                    self.status = Some(("Engraver connected".into(), false));
                    self.compiled = None; // the work area may differ
                }
                Event::Disconnected => {
                    self.connected = None;
                    self.head = None;
                    self.framing = None;
                    self.frame_req = None;
                    self.running = false;
                    self.ports_at = None;
                    self.logln("disconnected");
                }
                Event::State { head, framing } => {
                    self.head = head;
                    self.framing = framing;
                    if let (Some(f), Some((r, fp))) = (framing, self.frame_req)
                        && f == r
                    {
                        self.frame_req = None;
                        if fp.is_some() {
                            self.framed_fp = fp;
                        }
                    }
                }
                Event::Idle => {
                    self.pending = self.pending.saturating_sub(1);
                    if self.pending == 0 {
                        self.frame_req = None; // a failed frame request
                    }
                }
                Event::Progress(p) => self.progress = Some(p),
                Event::JobDone(res) => {
                    self.running = false;
                    let took = self.progress.as_ref().map(|p| fmt_dur(p.elapsed)).unwrap_or_default();
                    match res {
                        Ok(()) => self.info(format!("Job finished in {took}")),
                        Err(e) => self.fail(format!("Job {e}")),
                    }
                }
                Event::Timing(t) => {
                    self.timing = *t;
                    self.refresh_estimates();
                }
            }
        }
    }

    /// Follow edits with the framing beam.
    fn live_reframe(&mut self) {
        if self.running || self.pending > 0 || self.gesture.is_some() {
            return;
        }
        let Some(current) = self.framing else { return };
        match self.frame_target(self.frame_scope) {
            Some((rect, _)) if rect != current && self.last_change.elapsed() > Duration::from_millis(250) => {
                self.start_frame(self.frame_scope);
            }
            Some((rect, _)) if rect != current => {}
            Some((_, Some(fp))) => self.framed_fp = Some(fp),
            _ => {}
        }
    }

    // ------------------------------------------------------------ actions

    fn apply_actions(&mut self) {
        for a in std::mem::take(&mut self.act) {
            let editing = !self.running;
            match a {
                Action::AddFiles if editing => self.add_files_dialog(),
                Action::AddSource(src) if editing => self.add_op(Operation::new(src)),
                Action::AddTestGrid if editing => {
                    let ops = job::calibration_grid(&[20.0, 40.0, 60.0, 80.0, 100.0], &[5, 10, 20, 40, 80], 4.0, 1.5);
                    self.job.ops.extend(ops);
                    self.selected = None;
                    self.info(
                        "Test grid added: burn it on scrap, then pick the power and burn time of the best square",
                    );
                }
                Action::ReplaceFile(i) if editing => self.replace_file(i),
                Action::New if editing => self.request(Pending::New),
                Action::Open if editing => {
                    if let Some(p) = self.dialog().add_filter("k4laser project", &["toml"]).pick_file() {
                        self.request(Pending::Open(p));
                    }
                }
                Action::Save => self.save(None),
                Action::SaveAs => {
                    let name =
                        self.job_path.as_ref().and_then(|p| p.file_name()).map(|n| n.to_string_lossy().into_owned());
                    let d = self.dialog().add_filter("k4laser project", &["toml"]);
                    if let Some(p) = d.set_file_name(name.unwrap_or_else(|| "job.toml".into())).save_file() {
                        self.save(Some(p));
                    }
                }
                Action::Duplicate(i) if editing && i < self.job.ops.len() => {
                    let mut op = self.job.ops[i].clone();
                    op.x_mm = snap(op.x_mm + 2.0, P::PITCH_MM);
                    op.y_mm = snap(op.y_mm + 2.0, P::PITCH_MM);
                    self.job.ops.insert(i + 1, op);
                    self.selected = Some(i + 1);
                    self.frame_scope = FrameScope::Job;
                }
                Action::Delete(i) if editing && i < self.job.ops.len() => {
                    self.job.ops.remove(i);
                    let n = self.job.ops.len();
                    self.selected = (n > 0).then(|| i.min(n - 1));
                    self.frame_scope = FrameScope::Job;
                }
                Action::Reorder(i, up) if editing => {
                    let j = if up { i.checked_sub(1) } else { Some(i + 1) };
                    if let Some(j) = j.filter(|&j| j < self.job.ops.len() && i < self.job.ops.len()) {
                        self.job.ops.swap(i, j);
                        self.selected = Some(j);
                        self.frame_scope = FrameScope::Job;
                    }
                }
                Action::Undo if editing => self.undo(),
                Action::Redo if editing => self.redo(),
                Action::Connect(choice) => self.connect(choice),
                Action::Rescan => {
                    self.ports_at = None;
                    self.poll_ports();
                }
                Action::Frame(scope) if editing => self.start_frame(scope),
                Action::FrameStop => {
                    self.frame_req = None;
                    self.send(Cmd::FrameStop);
                }
                Action::Stop => self.stop(),
                Action::Center(i) if editing => {
                    let (aw, ah) = self.area_mm();
                    self.fixes.push(Fix { op: i, at: (aw / 2.0, ah / 2.0), frac: (0.5, 0.5), fit: false });
                }
                Action::Rotate(i, deg) if editing && i < self.job.ops.len() => {
                    let center = self.views.get(i).and_then(|v| v.frame).map(|[x, y, w, h]| (x + w / 2.0, y + h / 2.0));
                    let op = &mut self.job.ops[i];
                    op.rotate = (op.rotate + deg) % 360;
                    if let Some(at) = center {
                        self.fixes.push(Fix { op: i, at, frac: (0.5, 0.5), fit: false });
                    }
                }
                Action::Mirror(i) if editing && i < self.job.ops.len() => {
                    let op = &mut self.job.ops[i];
                    op.mirror = !op.mirror;
                }
                Action::Resize(i, fx, fy, frac) if editing && i < self.job.ops.len() => self.resize(i, fx, fy, frac),
                Action::Machine(cmd) => self.send(cmd),
                _ => {}
            }
        }
    }

    fn handle_drops(&mut self, ctx: &egui::Context) {
        let dropped: Vec<PathBuf> = ctx.input(|i| i.raw.dropped_files.iter().map(|f| f.path().to_path_buf()).collect());
        if self.running {
            return;
        }
        for p in dropped {
            self.add_path(&p);
        }
    }

    // ---------------------------------------------------------------- UI

    fn top_bar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.add_enabled_ui(!self.running, |ui| {
                let add = egui::Button::new(RichText::new("+  Add file…").strong()).min_size(Vec2::new(0.0, 28.0));
                if ui.add(add).on_hover_text("Image, SVG or DXF  (Ctrl+O)").clicked() {
                    self.act.push(Action::AddFiles);
                }
                ui.menu_button("Add ▾", |ui| {
                    let shape = |shape, w, h| Source::Shape(ShapeSrc { shape, width_mm: w, height_mm: h });
                    let items = [
                        ("Text", Source::default_of("text")),
                        ("QR code", Source::default_of("qr")),
                        ("Rectangle", Some(shape(ShapeKind::Rect, 30.0, 20.0))),
                        ("Circle", Some(shape(ShapeKind::Circle, 20.0, 20.0))),
                        ("Cross", Some(shape(ShapeKind::Cross, 10.0, 10.0))),
                        ("", None),
                        ("Test grid (power × burn time)", None),
                        ("Scale ruler", Source::default_of("ruler")),
                    ];
                    for (label, src) in items {
                        if label.is_empty() {
                            ui.separator();
                        } else if ui.button(label).clicked() {
                            self.act.push(src.map(Action::AddSource).unwrap_or(Action::AddTestGrid));
                            ui.close();
                        }
                    }
                });
                ui.separator();
                if ui.button("New").clicked() {
                    self.act.push(Action::New);
                }
                if ui.button("Open…").on_hover_text("Open a project (.toml)").clicked() {
                    self.act.push(Action::Open);
                }
            });
            if ui.button("Save").on_hover_text("Ctrl+S").clicked() {
                self.act.push(Action::Save);
            }
            if ui.button("Save as…").on_hover_text("Ctrl+Shift+S").clicked() {
                self.act.push(Action::SaveAs);
            }
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| self.machine_widget(ui));
        });
    }

    /// Right-to-left: the rightmost widget comes first.
    fn machine_widget(&mut self, ui: &mut egui::Ui) {
        let (aw, ah) = self.area_mm();
        let idle = !self.running && self.pending == 0;
        match &self.connected {
            Some((model, port)) => {
                let (model, port) = (model.describe(), port.clone());
                let text =
                    RichText::new(format!("●  Engraver connected · {aw:.0} × {ah:.0} mm ▾")).color(GREEN).strong();
                ui.menu_button(text, |ui| {
                    ui.label(RichText::new(model).weak());
                    ui.label(RichText::new(port).weak().small());
                    ui.separator();
                    if ui.add_enabled(idle, egui::Button::new("Home the head")).clicked() {
                        self.act.push(Action::Machine(Cmd::Home));
                        ui.close();
                    }
                    ui.menu_button("Fan / aux output (experimental)", |ui| {
                        ui.label(
                            RichText::new("Hidden vendor commands 0xFD / 0xFE.\nTheir effect is not confirmed.").weak(),
                        );
                        if ui.add_enabled(idle, egui::Button::new("On  (0xFD)")).clicked() {
                            self.act.push(Action::Machine(Cmd::Aux(true)));
                        }
                        if ui.add_enabled(idle, egui::Button::new("Off  (0xFE)")).clicked() {
                            self.act.push(Action::Machine(Cmd::Aux(false)));
                        }
                    });
                    ui.separator();
                    if ui.add_enabled(!self.running, egui::Button::new("Disconnect")).clicked() {
                        self.act.push(Action::Machine(Cmd::Disconnect));
                        ui.close();
                    }
                });
            }
            None => {
                ui.add_enabled_ui(idle, |ui| {
                    ui.menu_button(RichText::new("Connect ▾").strong(), |ui| {
                        ui.label(RichText::new("The engraver's USB serial port:").weak());
                        for p in &self.ports {
                            let path = p.stable_path().to_path_buf();
                            let mut text = p.label();
                            if p.is_ch340() {
                                text = format!("CH340  {text}");
                            }
                            if self.cfg.port.as_ref() == Some(&path) {
                                text.push_str("  ★ last used");
                            }
                            let r = ui
                                .add_enabled(p.holders.is_empty(), egui::Button::new(text))
                                .on_hover_text(path.display().to_string());
                            if r.clicked() {
                                self.act.push(Action::Connect(PortChoice::Serial(path)));
                                ui.close();
                            }
                        }
                        if self.ports.is_empty() {
                            ui.label("No serial port found: plug the engraver in.");
                        }
                        ui.separator();
                        if ui.button("Simulator (no hardware)").clicked() {
                            self.act.push(Action::Connect(PortChoice::Simulator));
                            ui.close();
                        }
                        if ui.button("Rescan").clicked() {
                            self.act.push(Action::Rescan);
                        }
                    });
                });
                ui.label(RichText::new("○  Engraver not connected").color(ORANGE));
            }
        }
        if self.pending > 0 && !self.running {
            ui.spinner();
        }
    }

    fn items_panel(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.heading("Items");
            if !self.job.ops.is_empty() {
                ui.label(RichText::new(self.job.ops.len().to_string()).weak());
            }
        });
        if self.job.ops.is_empty() {
            ui.label(RichText::new("Nothing yet. Add a file, or text, a QR code or a shape from the top bar.").weak());
            return;
        }
        egui::ScrollArea::vertical().id_salt("items").max_height(210.0).auto_shrink([false, true]).show(ui, |ui| {
            for i in 0..self.job.ops.len() {
                let err = self.views.get(i).is_some_and(|v| v.error.is_some());
                ui.horizontal(|ui| {
                    ui.checkbox(&mut self.job.ops[i].enabled, "").on_hover_text("Include in the job");
                    let op = &self.job.ops[i];
                    let power = if op.is_grayscale() {
                        format!("{:.0}–{:.0} %", op.min_power, op.power)
                    } else {
                        format!("{:.0} %", op.power)
                    };
                    let passes = if op.passes > 1 { format!(" ×{}", op.passes) } else { String::new() };
                    let summary = format!("{} · {power} · {}{passes}", op.mode.label(), op.depth);
                    let mut text = RichText::new(if err { format!("⚠ {}", op.label()) } else { op.label() });
                    if err {
                        text = text.color(ERROR);
                    } else if !op.enabled {
                        text = text.weak();
                    }
                    if ui.selectable_label(self.selected == Some(i), text).clicked() {
                        self.selected = Some(i);
                    }
                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        ui.label(RichText::new(summary).weak().small());
                    });
                });
            }
        });
        ui.horizontal(|ui| {
            let sel = self.selected.filter(|&i| i < self.job.ops.len());
            ui.add_enabled_ui(sel.is_some(), |ui| {
                let i = sel.unwrap_or(0);
                if ui.button("Duplicate").on_hover_text("Ctrl+D").clicked() {
                    self.act.push(Action::Duplicate(i));
                }
                if ui.button("Delete").on_hover_text("Del").clicked() {
                    self.act.push(Action::Delete(i));
                }
                if ui.button("▲").on_hover_text("Earlier in the job").clicked() {
                    self.act.push(Action::Reorder(i, true));
                }
                if ui.button("▼").on_hover_text("Later in the job").clicked() {
                    self.act.push(Action::Reorder(i, false));
                }
            });
        });
    }

    fn editor(&mut self, ui: &mut egui::Ui) {
        let Some(i) = self.selected.filter(|&i| i < self.job.ops.len()) else {
            ui.add_space(6.0);
            ui.label(RichText::new("Select an item to edit it.").weak());
            return;
        };
        let can_frame = self.connected.is_some() && !self.running && self.frame_target(FrameScope::Item(i)).is_some();
        let base_dir = self.job.base_dir.clone();
        let act = &mut self.act;
        let lock_ratio = &mut self.lock_ratio;
        let view = self.views.get(i);
        let frame = view.and_then(|v| v.frame);
        let op = &mut self.job.ops[i];
        ui.spacing_mut().slider_width = 170.0;

        let auto = {
            let mut o = op.clone();
            o.name.clear();
            o.label()
        };
        ui.horizontal(|ui| {
            ui.label(RichText::new(op.source.title()).strong().size(16.0));
            ui.add(egui::TextEdit::singleline(&mut op.name).hint_text(auto).desired_width(f32::INFINITY))
                .on_hover_text("Name in the item list");
        });

        // ---- what
        if let Some(path) = op.source.path().map(str::to_string) {
            ui.horizontal(|ui| {
                let shown = sources::resolve(&path, &base_dir);
                let name = shown.file_name().map(|n| n.to_string_lossy().into_owned()).unwrap_or(path.clone());
                ui.label(RichText::new(name).monospace()).on_hover_text(shown.display().to_string());
                if ui.small_button("Replace…").clicked() {
                    act.push(Action::ReplaceFile(i));
                }
            });
        }
        let modes = op.source.modes();
        if modes.len() > 1 {
            ui.horizontal(|ui| {
                ui.label("Mode");
                for &m in modes {
                    let hint = match m {
                        Mode::Raster => "Scan line by line and burn the filled areas (engraving)",
                        Mode::Vector => "Follow the paths with the beam (cutting, outlines)",
                    };
                    ui.selectable_value(&mut op.mode, m, m.label()).on_hover_text(hint);
                }
            });
        }
        match &mut op.source {
            Source::Image(s) => {
                ui.horizontal(|ui| {
                    for p in [ImageProcess::Threshold, ImageProcess::Dither, ImageProcess::Grayscale] {
                        let hint = match p {
                            ImageProcess::Threshold => "Pixels darker than the threshold burn. Logos, text, line art.",
                            ImageProcess::Dither => "Shades become dot density, in one pass. Photos.",
                            ImageProcess::Grayscale => "One pass per tone, each at its own power: real shades. Slower.",
                        };
                        ui.selectable_value(&mut s.process, p, p.label()).on_hover_text(hint);
                    }
                });
                egui::Grid::new("image").num_columns(2).spacing([10.0, 6.0]).show(ui, |ui| {
                    if s.process == ImageProcess::Threshold {
                        ui.label("Threshold");
                        ui.add(egui::Slider::new(&mut s.threshold, 0..=255))
                            .on_hover_text("Pixels darker than this burn");
                        ui.end_row();
                    }
                    if s.process == ImageProcess::Grayscale {
                        ui.label("Tones");
                        ui.add(egui::Slider::new(&mut s.levels, 2..=16)).on_hover_text("One pass per tone");
                        ui.end_row();
                    }
                    ui.label("Brightness");
                    ui.add(egui::Slider::new(&mut s.brightness, -100.0..=100.0).step_by(1.0));
                    ui.end_row();
                    ui.label("Contrast");
                    ui.add(egui::Slider::new(&mut s.contrast, -100.0..=100.0).step_by(1.0));
                    ui.end_row();
                    ui.label("");
                    ui.checkbox(&mut s.invert, "Invert (burn the light areas)");
                    ui.end_row();
                });
            }
            Source::Svg(s) => {
                if op.mode == Mode::Raster {
                    ui.checkbox(&mut s.invert, "Invert (burn the empty areas)");
                }
            }
            Source::Dxf(d) => {
                ui.horizontal(|ui| {
                    ui.label("Layers");
                    let mut layers = d.layers.join(", ");
                    let edit = egui::TextEdit::singleline(&mut layers).hint_text("all").desired_width(f32::INFINITY);
                    if ui.add(edit).changed() {
                        d.layers = layers.split(',').map(|s| s.trim().to_string()).filter(|s| !s.is_empty()).collect();
                    }
                });
            }
            Source::Text(t) => {
                ui.add(egui::TextEdit::multiline(&mut t.text).desired_rows(2).desired_width(f32::INFINITY));
                egui::Grid::new("text").num_columns(2).spacing([10.0, 6.0]).show(ui, |ui| {
                    ui.label("Font");
                    font_picker(ui, &mut t.font);
                    ui.end_row();
                    ui.label("Align");
                    ui.horizontal(|ui| {
                        ui.selectable_value(&mut t.align, Align::Left, "Left");
                        ui.selectable_value(&mut t.align, Align::Center, "Center");
                        ui.selectable_value(&mut t.align, Align::Right, "Right");
                    });
                    ui.end_row();
                });
            }
            Source::Qr(q) => {
                ui.add(
                    egui::TextEdit::multiline(&mut q.data)
                        .desired_rows(2)
                        .desired_width(f32::INFINITY)
                        .hint_text("Text or URL"),
                );
                ui.horizontal(|ui| {
                    ui.label("Error correction").on_hover_text("Share of the code that can be damaged");
                    for (l, pct) in [("L", "7 %"), ("M", "15 %"), ("Q", "25 %"), ("H", "30 %")] {
                        if ui.selectable_label(q.ecc.eq_ignore_ascii_case(l), l).on_hover_text(pct).clicked() {
                            q.ecc = l.into();
                        }
                    }
                });
                ui.label(RichText::new(format!("Module {:.2} mm", q.module_mm)).weak());
            }
            Source::Shape(s) => {
                ui.horizontal(|ui| {
                    ui.selectable_value(&mut s.shape, ShapeKind::Rect, "Rectangle");
                    ui.selectable_value(&mut s.shape, ShapeKind::Circle, "Circle");
                    ui.selectable_value(&mut s.shape, ShapeKind::Cross, "Cross");
                });
            }
            Source::Ruler(r) => {
                egui::Grid::new("ruler").num_columns(2).spacing([10.0, 6.0]).show(ui, |ui| {
                    ui.label("Tick step");
                    ui.add(mm_drag(&mut r.step_mm, 0.1..=10.0));
                    ui.end_row();
                    ui.label("");
                    ui.checkbox(&mut r.labels, "Labels every 10 mm");
                    ui.end_row();
                    ui.label("Label size");
                    ui.add(mm_drag(&mut r.label_mm, 0.5..=10.0));
                    ui.end_row();
                    ui.label("Font");
                    font_picker(ui, &mut r.font);
                    ui.end_row();
                });
            }
        }

        // ---- laser
        section(ui, "Laser");
        let gray = op.is_grayscale();
        egui::Grid::new("laser").num_columns(2).spacing([10.0, 6.0]).show(ui, |ui| {
            ui.label(if gray { "Max power" } else { "Power" });
            ui.add(egui::Slider::new(&mut op.power, 0.0..=100.0).step_by(0.5).suffix(" %")).on_hover_text(if gray {
                "Power of the darkest tone"
            } else {
                "Laser power"
            });
            ui.end_row();
            if gray {
                let max = op.power;
                ui.label("Min power");
                ui.add(egui::Slider::new(&mut op.min_power, 0.0..=max).step_by(0.5).suffix(" %"))
                    .on_hover_text("Power of the lightest tone");
                ui.end_row();
            }
            ui.label("Burn time").on_hover_text(DEPTH_HINT);
            ui.add(egui::Slider::new(&mut op.depth, 0..=P::DEPTH_MAX)).on_hover_text(DEPTH_HINT);
            ui.end_row();
            ui.label("Passes");
            ui.add(egui::DragValue::new(&mut op.passes).range(1..=50))
                .on_hover_text("Repeat the whole item: more passes cut deeper");
            ui.end_row();
        });
        op.min_power = op.min_power.clamp(0.0, op.power);

        if op.mode == Mode::Vector {
            section(ui, "Outlines");
            egui::Grid::new("outlines").num_columns(2).spacing([10.0, 6.0]).show(ui, |ui| {
                ui.label("Count").on_hover_text(OUTLINE_HINT);
                ui.add(egui::DragValue::new(&mut op.outlines).range(1..=50)).on_hover_text(OUTLINE_HINT);
                ui.end_row();
                if op.outlines > 1 {
                    ui.label("Spacing");
                    ui.add(
                        egui::DragValue::new(&mut op.outline_spacing_mm)
                            .speed(0.005)
                            .range(0.02..=2.0)
                            .fixed_decimals(2)
                            .suffix(" mm"),
                    )
                    .on_hover_text("Distance between outlines; about the beam width");
                    ui.end_row();
                    ui.label("Side");
                    ui.horizontal(|ui| {
                        for (side, hint) in [
                            (Side::Outside, "Away from the material: a cut-out part keeps its exact size"),
                            (Side::Inside, "Into the material: a hole keeps its exact size"),
                            (Side::Centered, "Centred on the path"),
                        ] {
                            ui.selectable_value(&mut op.outline_side, side, side.label()).on_hover_text(hint);
                        }
                    });
                    ui.end_row();
                }
            });
        }

        // ---- placement
        section(ui, "Size and position");
        if let Some([x, y, w, h]) = frame {
            let forced = op.source.keeps_aspect();
            let locked = forced || *lock_ratio;
            egui::Grid::new("place").num_columns(2).spacing([10.0, 6.0]).show(ui, |ui| {
                ui.label("Size");
                ui.horizontal(|ui| {
                    let (mut nw, mut nh) = (w, h);
                    let rw = ui.add(mm_drag(&mut nw, 0.1..=1000.0));
                    ui.label("×");
                    let rh = ui.add(mm_drag(&mut nh, 0.1..=1000.0));
                    if rw.changed() && w > 0.0 {
                        let f = nw / w;
                        act.push(Action::Resize(i, f, if locked { f } else { 1.0 }, (0.0, 0.0)));
                    } else if rh.changed() && h > 0.0 {
                        let f = nh / h;
                        act.push(Action::Resize(i, if locked { f } else { 1.0 }, f, (0.0, 0.0)));
                    }
                    let hint = if forced { "This item always keeps its proportions" } else { "Keep proportions" };
                    let lock =
                        ui.add_enabled(!forced, egui::Button::selectable(locked, "Keep ratio")).on_hover_text(hint);
                    if lock.clicked() {
                        *lock_ratio = !*lock_ratio;
                    }
                });
                ui.end_row();
                ui.label("Position").on_hover_text("Top-left corner, from the top-left of the bed");
                ui.horizontal(|ui| {
                    let (mut nx, mut ny) = (x, y);
                    ui.label("X");
                    if ui.add(mm_drag(&mut nx, -1000.0..=1000.0)).changed() {
                        op.x_mm = snap(op.x_mm + nx - x, P::PITCH_MM);
                    }
                    ui.label("Y");
                    if ui.add(mm_drag(&mut ny, -1000.0..=1000.0)).changed() {
                        op.y_mm = snap(op.y_mm + ny - y, P::PITCH_MM);
                    }
                });
                ui.end_row();
            });
            ui.horizontal(|ui| {
                if ui.button("Center").on_hover_text("Center on the bed").clicked() {
                    act.push(Action::Center(i));
                }
                if ui.button("↺ 90°").on_hover_text("Rotate counter-clockwise").clicked() {
                    act.push(Action::Rotate(i, 270));
                }
                if ui.button("↻ 90°").on_hover_text("Rotate clockwise").clicked() {
                    act.push(Action::Rotate(i, 90));
                }
                if ui.selectable_label(op.mirror, "⇆ Mirror").on_hover_text("Mirror left-right").clicked() {
                    act.push(Action::Mirror(i));
                }
            });
        } else {
            ui.label(RichText::new("Nothing to place.").weak());
        }

        // ---- details
        section(ui, "Details");
        if let Some(v) = view {
            let rows: usize = v.plans.iter().map(OpPlan::frame_count).sum();
            let (burn, segs) = v.plans.iter().fold((0.0, 0), |(b, s), p| match &*p.geometry {
                Geometry::Vector(g) => (b + p.lengths_mm().0 * p.passes as f64, s + g.len()),
                Geometry::Raster(_) => (b, s),
            });
            if segs > 0 {
                ui.label(RichText::new(format!("{segs} segments, {burn:.0} mm of line in total")).weak());
            } else if rows > 0 {
                let parts = if v.plans.len() > 1 { format!(" in {} tones", v.plans.len()) } else { String::new() };
                ui.label(RichText::new(format!("{rows} lines to scan{parts}")).weak());
            }
            if let Some(d) = &v.duration {
                ui.label(RichText::new(format!("Duration {d}")).weak()).on_hover_text(timing_hint(&self.timing));
            }
            for n in &v.notes {
                ui.label(RichText::new(format!("• {n}")).weak());
            }
            if let Some(e) = &v.error {
                ui.colored_label(ERROR, e);
            }
        }
        ui.add_space(4.0);
        let frame_btn = egui::Button::new("Frame this item");
        if ui
            .add_enabled(can_frame, frame_btn)
            .on_hover_text("Trace this item's outline with the low-power beam")
            .on_disabled_hover_text("Connect the engraver first")
            .clicked()
        {
            act.push(Action::Frame(FrameScope::Item(i)));
        }
    }

    fn head_panel(&mut self, ui: &mut egui::Ui) {
        let idle = self.connected.is_some() && !self.running && self.pending == 0;
        egui::CollapsingHeader::new("Laser head").default_open(false).show(ui, |ui| {
            ui.horizontal(|ui| {
                if ui.add_enabled(idle, egui::Button::new("Home")).clicked() {
                    self.act.push(Action::Machine(Cmd::Home));
                }
                let head = match self.head {
                    Some((x, y)) => format!("at {:.2}, {:.2} mm", x as f64 * P::PITCH_MM, y as f64 * P::PITCH_MM),
                    None => "position unknown (homes before moving)".into(),
                };
                ui.label(RichText::new(head).weak());
            });
            let step = (self.jog_mm / P::PITCH_MM).round() as i64;
            ui.horizontal(|ui| {
                ui.add_enabled_ui(idle, |ui| {
                    for (label, d) in [("←", (-step, 0)), ("↑", (0, -step)), ("↓", (0, step)), ("→", (step, 0))]
                    {
                        if ui.button(label).clicked() {
                            self.act.push(Action::Machine(Cmd::Jog(d.0, d.1)));
                        }
                    }
                });
                egui::ComboBox::from_id_salt("jog").width(80.0).selected_text(format!("{} mm", self.jog_mm)).show_ui(
                    ui,
                    |ui| {
                        for s in [0.05, 0.1, 0.5, 1.0, 5.0, 10.0] {
                            ui.selectable_value(&mut self.jog_mm, s, format!("{s} mm"));
                        }
                    },
                );
            });
        });
    }

    fn bottom_bar(&mut self, ui: &mut egui::Ui) {
        let connected = self.connected.is_some();
        let framing = self.framing_active();
        let ready = self.can_start();
        let can_frame = connected && !self.running && (framing || self.frame_target(FrameScope::Job).is_some());
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 10.0;
            let size = Vec2::new(130.0, 42.0);
            let stop = egui::Button::new(RichText::new("■  STOP").size(18.0).strong().color(Color32::WHITE))
                .fill(RED)
                .min_size(size);
            if ui.add_enabled(connected, stop).on_hover_text("Esc. Aborts the job, switches the beam off.").clicked() {
                self.act.push(Action::Stop);
            }
            let frame = if framing {
                egui::Button::new(RichText::new("■  Stop framing").size(16.0).color(Color32::WHITE)).fill(ORANGE)
            } else {
                egui::Button::new(RichText::new("◻  Frame").size(16.0))
            };
            let hint = "Trace the outline of the job with the low-power beam, to align the piece. It follows your \
                        edits while on.";
            if ui.add_enabled(can_frame, frame.min_size(size)).on_hover_text(hint).clicked() {
                self.act.push(if framing { Action::FrameStop } else { Action::Frame(FrameScope::Job) });
            }
            let start = egui::Button::new(RichText::new("▶  Start").size(18.0).strong().color(Color32::WHITE))
                .fill(GREEN)
                .min_size(size);
            if ui.add_enabled(ready.is_ok(), start).clicked() {
                self.confirm = true;
            }
            if self.running {
                let paused = self.control.paused.load(Ordering::SeqCst);
                let label = if paused { "Resume" } else { "Pause" };
                let pause = egui::Button::new(RichText::new(label).size(16.0)).min_size(Vec2::new(100.0, 42.0));
                if ui.add(pause).on_hover_text("Holds the next line on the host side").clicked() {
                    self.control.paused.store(!paused, Ordering::SeqCst);
                }
            }
            ui.vertical(|ui| {
                ui.set_max_width((ui.available_width() - 70.0).max(100.0));
                if self.running {
                    if let Some(p) = &self.progress {
                        let frac = p.step as f32 / p.total.max(1) as f32;
                        let eta = p.eta.map(|e| format!(" · {} left", fmt_dur(e))).unwrap_or_default();
                        let text = format!("{:.0} % · {}{eta}", frac * 100.0, fmt_dur(p.elapsed));
                        ui.add(egui::ProgressBar::new(frac).text(text));
                        ui.label(RichText::new(&p.label).weak().small());
                    } else {
                        ui.add(egui::ProgressBar::new(0.0).text("starting…").animate(true));
                    }
                } else {
                    ui.horizontal(|ui| {
                        if let Err(reason) = &ready {
                            ui.label(RichText::new(reason).color(ORANGE));
                        } else if self.framed_fp != Some(self.plan_fp) {
                            ui.label(RichText::new("Ready. Frame first to check the position.").weak());
                        } else {
                            ui.label(RichText::new("Ready").color(GREEN));
                        }
                        if let Some(e) = &self.estimate {
                            ui.label(RichText::new(format!("·  duration {e}")).weak())
                                .on_hover_text(timing_hint(&self.timing));
                        }
                    });
                    if let Some((s, err)) = &self.status {
                        ui.label(RichText::new(s).color(if *err { ERROR } else { ui.visuals().weak_text_color() }));
                    }
                }
            });
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui.selectable_label(self.show_log, "Log").clicked() {
                    self.show_log = !self.show_log;
                }
            });
        });
    }

    fn log_panel(&mut self, ui: &mut egui::Ui) {
        egui::ScrollArea::vertical().id_salt("log").stick_to_bottom(true).auto_shrink([false, false]).show(ui, |ui| {
            for line in self.log.iter().rev().take(300).rev() {
                let mut t = RichText::new(line).monospace().size(11.0);
                if line.starts_with("error") {
                    t = t.color(ERROR);
                }
                ui.label(t);
            }
        });
    }

    fn confirm_modal(&mut self, ctx: &egui::Context) {
        if !self.confirm {
            return;
        }
        if self.can_start().is_err() && !self.framing_active() {
            self.confirm = false;
            return;
        }
        let modal = egui::Modal::new(egui::Id::unique("confirm-start")).show(ctx, |ui| {
            ui.set_width(480.0);
            ui.heading("Start engraving?");
            ui.add_space(6.0);
            let p = &self.plan;
            let items = self.job.ops.iter().zip(&self.views).filter(|(o, v)| o.enabled && !v.plans.is_empty()).count();
            if let Some(b) = p.bbox() {
                let mm = |v: u32| v as f64 * P::PITCH_MM;
                ui.label(format!(
                    "{items} item(s), {:.2} × {:.2} mm at ({:.2}, {:.2}) mm",
                    mm(b.w),
                    mm(b.h),
                    mm(b.x),
                    mm(b.y)
                ));
            }
            if let Some(e) = &self.estimate {
                ui.label(format!("Duration {e}")).on_hover_text(timing_hint(&self.timing));
            }
            egui::CollapsingHeader::new("Details").default_open(false).show(ui, |ui| {
                for line in p.summary() {
                    ui.label(RichText::new(line).monospace().size(11.0));
                }
            });
            ui.add_space(6.0);
            let mut frame_first = false;
            if self.frame_req.is_some() {
                ui.horizontal(|ui| {
                    ui.spinner();
                    ui.label("Framing…");
                });
            } else if self.framed_fp != Some(self.plan_fp) {
                ui.horizontal(|ui| {
                    ui.label(RichText::new("⚠ Not framed since the last change.").color(ORANGE));
                    frame_first = ui.button("Frame first").clicked();
                });
            } else if self.framing.is_some() {
                ui.label(RichText::new("Framed. The beam stops tracing when the job starts.").weak());
            }
            ui.add_space(6.0);
            ui.label(
                RichText::new("Eye protection on, piece secured, fumes extracted. STOP or Esc aborts at any time.")
                    .weak(),
            );
            ui.add_space(8.0);
            let mut go = false;
            let mut cancel = false;
            ui.horizontal(|ui| {
                let start = egui::Button::new(RichText::new("▶  Start").size(16.0).strong().color(Color32::WHITE))
                    .fill(GREEN)
                    .min_size(Vec2::new(120.0, 34.0));
                let enabled = self.pending == 0;
                go = ui.add_enabled(enabled, start).on_hover_text("Enter").clicked()
                    || (enabled && ui.input(|i| i.key_pressed(Key::Enter)));
                cancel = ui.add(egui::Button::new("Cancel").min_size(Vec2::new(90.0, 34.0))).clicked();
            });
            (go, cancel, frame_first)
        });
        let (go, cancel, frame_first) = modal.inner;
        if frame_first {
            self.start_frame(FrameScope::Job);
        }
        if go {
            self.start_job();
        } else if cancel || modal.should_close() {
            self.confirm = false;
        }
    }

    fn discard_modal(&mut self, ctx: &egui::Context) {
        let Some(pending) = &self.discard else { return };
        let what = match pending {
            Pending::New => "start a new project",
            Pending::Open(_) => "open another project",
        };
        let name = self
            .job_path
            .as_ref()
            .and_then(|p| p.file_name())
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_else(|| "the current project".into());
        let modal = egui::Modal::new(egui::Id::unique("discard")).show(ctx, |ui| {
            ui.set_width(400.0);
            ui.heading("Unsaved changes");
            ui.label(format!("Save the changes to {name} before you {what}?"));
            ui.add_space(8.0);
            let mut choice = 0;
            ui.horizontal(|ui| {
                if ui.button("Save").clicked() {
                    choice = 1;
                }
                if ui.button("Discard changes").clicked() {
                    choice = 2;
                }
                if ui.button("Cancel").clicked() {
                    choice = 3;
                }
            });
            choice
        });
        match modal.inner {
            1 => {
                self.save(None);
                if !self.dirty()
                    && let Some(p) = self.discard.take()
                {
                    self.proceed(p);
                }
            }
            2 => {
                if let Some(p) = self.discard.take() {
                    self.proceed(p);
                }
            }
            3 => self.discard = None,
            _ if modal.should_close() => self.discard = None,
            _ => {}
        }
    }

    // ------------------------------------------------------------ canvas

    fn canvas(&mut self, ui: &mut egui::Ui) {
        let (resp, painter) = ui.allocate_painter(ui.available_size(), Sense::click_and_drag());
        let rect = resp.rect;
        let dark = ui.visuals().dark_mode;
        painter.rect_filled(rect, 0.0, if dark { Color32::from_gray(30) } else { Color32::from_gray(214) });
        let (aw, ah) = self.area_mm();
        // always fitted and centred: the whole bed is visible at any window size
        let avail = egui::Rect::from_min_max(rect.min + Vec2::new(42.0, 28.0), rect.max - Vec2::new(18.0, 30.0));
        let z = (avail.width() as f64 / aw).min(avail.height() as f64 / ah).max(0.1);
        let bed = egui::Rect::from_center_size(avail.center(), Vec2::new((aw * z) as f32, (ah * z) as f32));
        let tf = Tf { origin: bed.min, z };
        painter.rect_filled(bed, 0.0, BED);

        // grid: 1 mm when legible, 5 mm, 10 mm with labels
        let (minor, mid, major) =
            (Color32::from_rgb(236, 231, 221), Color32::from_rgb(222, 214, 200), Color32::from_rgb(198, 188, 170));
        let label_col = if dark { Color32::from_gray(150) } else { Color32::from_gray(90) };
        let font = egui::FontId::monospace(10.0);
        for (len, vertical) in [(aw, true), (ah, false)] {
            for mm in 0..=(len.floor() as i32) {
                let col = match mm {
                    _ if mm % 10 == 0 => major,
                    _ if mm % 5 == 0 => mid,
                    _ if z >= 5.0 => minor,
                    _ => continue,
                };
                let v = mm as f64;
                let (a, b) = if vertical { (tf.pos(v, 0.0), tf.pos(v, ah)) } else { (tf.pos(0.0, v), tf.pos(aw, v)) };
                painter.line_segment([a, b], Stroke::new(1.0, col));
                if mm % 10 == 0 {
                    let (p, align) = if vertical {
                        (a - Vec2::new(0.0, 4.0), Align2::CENTER_BOTTOM)
                    } else {
                        (a - Vec2::new(5.0, 0.0), Align2::RIGHT_CENTER)
                    };
                    painter.text(p, align, mm.to_string(), font.clone(), label_col);
                }
            }
        }
        painter.rect_stroke(bed, 0.0, Stroke::new(1.0, major), egui::StrokeKind::Outside);
        painter.text(
            bed.right_bottom() + Vec2::new(0.0, 4.0),
            Align2::RIGHT_TOP,
            format!("bed {aw:.0} × {ah:.0} mm"),
            egui::FontId::proportional(11.0),
            label_col,
        );

        // ---- items
        let painter = painter.with_clip_rect(rect);
        let preview = match &self.gesture {
            Some(Gesture::Resize { op, frame, corner, scale }) => Some((*op, *frame, resized(*frame, *corner, *scale))),
            _ => None,
        };
        let pitch = P::PITCH_MM;
        let mut used = Vec::new();
        for (i, (op, v)) in self.job.ops.iter().zip(&self.views).enumerate() {
            // bed mm -> screen, with a resize preview applied
            let map = |x: f64, y: f64| match preview {
                Some((j, [fx, fy, fw, fh], [nx, ny, nw, nh])) if j == i => {
                    tf.pos(nx + (x - fx) * nw / fw.max(1e-9), ny + (y - fy) * nh / fh.max(1e-9))
                }
                _ => tf.pos(x, y),
            };
            let faded = !op.enabled;
            for p in &v.plans {
                let (x0, y0) = (p.x as f64 * pitch, p.y as f64 * pitch);
                match &*p.geometry {
                    Geometry::Raster(bm) => {
                        if bm.width > 8192 || bm.height > 8192 {
                            continue;
                        }
                        let tex = self.textures.entry(p.geom_hash).or_insert_with(|| {
                            let pixels = bm
                                .data
                                .iter()
                                .map(|&b| if b { Color32::WHITE } else { Color32::TRANSPARENT })
                                .collect();
                            let img = egui::ColorImage::new([bm.width, bm.height], pixels);
                            ui.ctx().load_texture(format!("op-{:x}", p.geom_hash), img, egui::TextureOptions::NEAREST)
                        });
                        used.push(p.geom_hash);
                        let r = egui::Rect::from_min_max(
                            map(x0, y0),
                            map(x0 + p.width as f64 * pitch, y0 + p.height as f64 * pitch),
                        );
                        let uv = egui::Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0));
                        painter.image(tex.id(), r, uv, burn_tint(p.power, faded));
                    }
                    Geometry::Vector(segs) => {
                        let stroke = Stroke::new(1.2, if faded { LINE.gamma_multiply(0.25) } else { LINE });
                        for s in segs {
                            let a = map(x0 + s[0] as f64 * pitch, y0 + s[1] as f64 * pitch);
                            let b = map(x0 + s[2] as f64 * pitch, y0 + s[3] as f64 * pitch);
                            painter.line_segment([a, b], stroke);
                        }
                    }
                }
            }
            if v.error.is_some()
                && let Some(f) = v.frame
            {
                dashed_rect(&painter, tf.rect(f), Stroke::new(1.5, ERROR));
            }
        }
        self.textures.retain(|k, _| used.contains(k));

        // ---- interaction
        let editable = !self.running;
        let sel = self.selected.filter(|&i| i < self.views.len());
        let item_box = |i: usize| -> Option<[f64; 4]> {
            match preview {
                Some((j, _, r)) if j == i => Some(r),
                _ => self.views.get(i)?.frame,
            }
        };
        let handle_at = |pos: Pos2| -> Option<(f64, f64)> {
            let r = tf.rect(item_box(sel?)?);
            [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)].into_iter().find(|&(cx, cy)| {
                let c =
                    Pos2::new(if cx > 0.5 { r.right() } else { r.left() }, if cy > 0.5 { r.bottom() } else { r.top() });
                (c - pos).length() <= 9.0
            })
        };
        let item_at = |pos: Pos2| -> Option<usize> {
            let inside = |i: usize| item_box(i).is_some_and(|f| tf.rect(f).expand(3.0).contains(pos));
            // the selection first, so overlapped items stay reachable
            sel.filter(|&i| inside(i)).or_else(|| (0..self.views.len()).rev().find(|&i| inside(i)))
        };
        let pointer = resp.hover_pos();
        let hovered = pointer.and_then(item_at);
        let hovered_handle = pointer.and_then(handle_at);

        // selection and hover outlines
        if let Some(i) = hovered.filter(|&i| Some(i) != sel && self.gesture.is_none())
            && let Some(f) = item_box(i)
        {
            dashed_rect(&painter, tf.rect(f), Stroke::new(1.0, BLUE));
        }
        if let Some(i) = sel
            && let Some(f) = item_box(i)
        {
            let r = tf.rect(f);
            painter.rect_stroke(r, 0.0, Stroke::new(1.5, BLUE), egui::StrokeKind::Outside);
            if editable {
                for c in [r.left_top(), r.right_top(), r.right_bottom(), r.left_bottom()] {
                    let h = egui::Rect::from_center_size(c, Vec2::splat(8.0));
                    painter.rect_filled(h, 1.0, Color32::WHITE);
                    painter.rect_stroke(h, 1.0, Stroke::new(1.5, BLUE), egui::StrokeKind::Inside);
                }
            }
            painter.text(
                r.left_top() + Vec2::new(0.0, -6.0),
                Align2::LEFT_BOTTOM,
                format!("{:.2} × {:.2} mm", f[2], f[3]),
                egui::FontId::proportional(11.0),
                BLUE,
            );
        }

        // live framing rectangle and head
        if let Some(f) = self.framing {
            let r = tf.rect([f.x as f64 * pitch, f.y as f64 * pitch, f.w as f64 * pitch, f.h as f64 * pitch]);
            painter.rect_stroke(r, 0.0, Stroke::new(2.5, BEAM), egui::StrokeKind::Middle);
            painter.text(
                r.left_bottom() + Vec2::new(0.0, 4.0),
                Align2::LEFT_TOP,
                "framing",
                egui::FontId::proportional(11.0),
                BEAM,
            );
        }
        if let Some((hx, hy)) = self.head {
            let c = tf.pos(hx as f64 * pitch, hy as f64 * pitch);
            let s = Stroke::new(1.5, Color32::from_rgb(20, 110, 200));
            painter.line_segment([c - Vec2::new(9.0, 0.0), c + Vec2::new(9.0, 0.0)], s);
            painter.line_segment([c - Vec2::new(0.0, 9.0), c + Vec2::new(0.0, 9.0)], s);
            painter.circle_stroke(c, 4.0, s);
        }

        // cursor
        if editable && pointer.is_some() {
            let corner_icon = |(cx, cy): (f64, f64)| {
                if (cx > 0.5) == (cy > 0.5) { egui::CursorIcon::ResizeNwSe } else { egui::CursorIcon::ResizeNeSw }
            };
            let icon = match &self.gesture {
                Some(Gesture::Move { .. }) => Some(egui::CursorIcon::Grabbing),
                Some(Gesture::Resize { corner, .. }) => Some(corner_icon(*corner)),
                None => hovered_handle.map(corner_icon).or(hovered.map(|_| egui::CursorIcon::Grab)),
            };
            if let Some(icon) = icon {
                ui.ctx().set_cursor_icon(icon);
            }
        }

        // gestures
        let shift = ui.input(|i| i.modifiers.shift);
        if editable
            && resp.drag_started_by(egui::PointerButton::Primary)
            && let Some(pos) = ui.input(|i| i.pointer.press_origin())
        {
            if let (Some(i), Some(corner)) = (sel, handle_at(pos))
                && let Some(frame) = item_box(i)
            {
                self.gesture = Some(Gesture::Resize { op: i, frame, corner, scale: (1.0, 1.0) });
            } else if let Some(i) = item_at(pos)
                && let Some(frame) = item_box(i)
            {
                self.selected = Some(i);
                let op = &self.job.ops[i];
                self.gesture = Some(Gesture::Move { op: i, start: (op.x_mm, op.y_mm), frame, grab: tf.mm(pos) });
            }
        }
        if let Some(pos) = resp.interact_pointer_pos().filter(|_| resp.dragged()) {
            let (mx, my) = tf.mm(pos);
            match &mut self.gesture {
                Some(Gesture::Move { op, start, frame, grab }) if *op < self.job.ops.len() => {
                    let step = if shift { 1.0 } else { pitch };
                    let [fx, fy, fw, fh] = *frame;
                    // keep the item on the bed when it fits
                    let clamp = |v: f64, lo: f64, hi: f64| if lo <= hi { v.clamp(lo, hi) } else { v };
                    let o = &mut self.job.ops[*op];
                    o.x_mm = clamp(snap(start.0 + mx - grab.0, step), start.0 - fx, start.0 + aw - fw - fx);
                    o.y_mm = clamp(snap(start.1 + my - grab.1, step), start.1 - fy, start.1 + ah - fh - fy);
                }
                Some(Gesture::Resize { op, frame, corner, scale }) if *op < self.job.ops.len() => {
                    let [x, y, w, h] = *frame;
                    let (cx, cy) = *corner;
                    let anchor = (x + (1.0 - cx) * w, y + (1.0 - cy) * h);
                    let dir = |c: f64| if c > 0.5 { 1.0 } else { -1.0 };
                    let (dw, dh) = ((mx - anchor.0) * dir(cx), (my - anchor.1) * dir(cy));
                    let locked = self.lock_ratio || self.job.ops[*op].source.keeps_aspect();
                    let (sx, sy) = if locked {
                        // project on the diagonal: the box follows the pointer smoothly
                        let f = (dw * w + dh * h) / (w * w + h * h).max(1e-9);
                        (f, f)
                    } else {
                        (dw / w.max(1e-9), dh / h.max(1e-9))
                    };
                    let min = 0.5 / w.min(h).max(1e-9);
                    *scale = (sx.max(min), sy.max(min));
                }
                _ => {}
            }
        }
        if resp.drag_stopped() {
            if let Some(Gesture::Resize { op, corner, scale, .. }) = self.gesture.take()
                && ((scale.0 - 1.0).abs() > 1e-4 || (scale.1 - 1.0).abs() > 1e-4)
            {
                self.act.push(Action::Resize(op, scale.0, scale.1, (1.0 - corner.0, 1.0 - corner.1)));
            }
            self.gesture = None;
        }
        if resp.clicked()
            && let Some(pos) = resp.interact_pointer_pos()
        {
            self.selected = item_at(pos);
        }

        // pointer position
        if let Some(pos) = pointer {
            let (x, y) = tf.mm(pos);
            painter.text(
                rect.left_bottom() + Vec2::new(10.0, -8.0),
                Align2::LEFT_BOTTOM,
                format!("X {x:6.2}   Y {y:6.2} mm"),
                egui::FontId::monospace(12.0),
                label_col,
            );
        }

        // empty bed
        if self.job.ops.is_empty() {
            let r = egui::Rect::from_center_size(bed.center(), Vec2::new(300.0, 54.0));
            let add = egui::Button::new(RichText::new("+  Add a file to engrave…").size(18.0).strong());
            if ui.put(r, add).clicked() {
                self.act.push(Action::AddFiles);
            }
            painter.text(
                r.center_bottom() + Vec2::new(0.0, 14.0),
                Align2::CENTER_TOP,
                "Image (PNG, JPEG…), SVG or DXF.  Text, QR codes and shapes: Add ▾",
                egui::FontId::proportional(13.0),
                Color32::from_gray(110),
            );
        }
        if ui.input(|i| !i.raw.hovered_files.is_empty()) {
            painter.rect_filled(rect, 0.0, Color32::from_black_alpha(110));
            painter.text(
                rect.center(),
                Align2::CENTER_CENTER,
                "Drop to add",
                egui::FontId::proportional(26.0),
                Color32::WHITE,
            );
        }
    }

    fn keyboard(&mut self, ctx: &egui::Context) {
        if ctx.input(|i| i.key_pressed(Key::Escape)) && (self.running || self.framing_active()) {
            self.stop();
        }
        if ctx.egui_wants_keyboard_input() || self.confirm || self.discard.is_some() {
            return;
        }
        let sc = |m: Modifiers, k: Key| ctx.input_mut(|i| i.consume_shortcut(&KeyboardShortcut::new(m, k)));
        if sc(Modifiers::COMMAND | Modifiers::SHIFT, Key::S) {
            self.act.push(Action::SaveAs);
        }
        if sc(Modifiers::COMMAND, Key::S) {
            self.act.push(Action::Save);
        }
        if self.running {
            return;
        }
        if sc(Modifiers::COMMAND | Modifiers::SHIFT, Key::Z) || sc(Modifiers::COMMAND, Key::Y) {
            self.act.push(Action::Redo);
        }
        if sc(Modifiers::COMMAND, Key::Z) {
            self.act.push(Action::Undo);
        }
        if sc(Modifiers::COMMAND, Key::O) {
            self.act.push(Action::AddFiles);
        }
        let Some(i) = self.selected.filter(|&i| i < self.job.ops.len()) else { return };
        if sc(Modifiers::COMMAND, Key::D) {
            self.act.push(Action::Duplicate(i));
        }
        if ctx.input(|inp| inp.key_pressed(Key::Delete) || inp.key_pressed(Key::Backspace)) {
            self.act.push(Action::Delete(i));
        }
        let (dx, dy, shift) = ctx.input(|inp| {
            let k = |key| if inp.key_pressed(key) { 1.0 } else { 0.0 };
            (k(Key::ArrowRight) - k(Key::ArrowLeft), k(Key::ArrowDown) - k(Key::ArrowUp), inp.modifiers.shift)
        });
        if dx != 0.0 || dy != 0.0 {
            let step = if shift { 1.0 } else { P::PITCH_MM };
            let op = &mut self.job.ops[i];
            op.x_mm = snap(op.x_mm + dx * step, P::PITCH_MM);
            op.y_mm = snap(op.y_mm + dy * step, P::PITCH_MM);
        }
    }

    /// Remember the laser settings of the selected item for the next new one.
    fn track_laser(&mut self) {
        if let Some(op) = self.selected.and_then(|i| self.job.ops.get(i)) {
            self.laser = Laser {
                power: op.power,
                min_power: if op.is_grayscale() { op.min_power } else { self.laser.min_power },
                depth: op.depth,
                passes: op.passes,
            };
        }
    }

    fn update_title(&mut self, ctx: &egui::Context) {
        let name = self
            .job_path
            .as_ref()
            .and_then(|p| p.file_name())
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_else(|| "untitled".into());
        let dirty = if self.dirty() { " •" } else { "" };
        let title = format!("{name}{dirty} — k4laser");
        if title != self.title {
            ctx.send_viewport_cmd(egui::ViewportCommand::Title(title.clone()));
            self.title = title;
        }
    }

    /// `K4LASER_SCREENSHOT=out.png k4laser gui job.toml`: render, save a screenshot, quit.
    /// With `K4LASER_SELFTEST_SIM=1`, also connect to the simulator and frame the job.
    fn screenshot_self_test(&mut self, ctx: &egui::Context) {
        let Some((path, frames)) = self.screenshot.as_mut() else { return };
        *frames += 1;
        let (n, path) = (*frames, path.clone());
        if n == 2 && !self.job.ops.is_empty() {
            let i = std::env::var("K4LASER_SELECT").ok().and_then(|s| s.parse().ok()).unwrap_or(0);
            self.selected = Some(i);
        }
        if std::env::var_os("K4LASER_SELFTEST_SIM").is_some() {
            if n == 2 {
                self.connect(PortChoice::Simulator);
            }
            if n < 25 && self.connected.is_some() && self.pending == 0 && !self.framing_active() {
                self.start_frame(FrameScope::Job);
            }
        }
        if n == 30 {
            let ctx2 = ctx.clone();
            ctx.send_viewport_cmd(egui::ViewportCommand::Screenshot(egui::ScreenshotCallback::new(move |img| {
                let [w, h] = img.size;
                let raw: Vec<u8> = img.pixels.iter().flat_map(|c| c.to_array()).collect();
                if let Some(buf) = image::RgbaImage::from_raw(w as u32, h as u32, raw) {
                    let _ = buf.save(&path);
                }
                ctx2.send_viewport_cmd(egui::ViewportCommand::Close);
                ctx2.request_repaint();
            })));
        }
        if n <= 30 {
            ctx.request_repaint();
        }
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        self.drain_events();
        self.poll_ports();
        self.handle_drops(&ctx);
        self.keyboard(&ctx);
        self.apply_actions();
        self.settle();

        egui::Panel::top("top").show(ui, |ui| {
            ui.add_space(5.0);
            self.top_bar(ui);
            ui.add_space(3.0);
        });
        egui::Panel::bottom("actions").show(ui, |ui| {
            ui.add_space(6.0);
            self.bottom_bar(ui);
            ui.add_space(4.0);
        });
        if self.show_log {
            egui::Panel::bottom("log").resizable(true).default_size(140.0).show(ui, |ui| self.log_panel(ui));
        }
        egui::Panel::right("side").resizable(true).default_size(380.0).show(ui, |ui| {
            ui.add_space(4.0);
            ui.add_enabled_ui(!self.running, |ui| {
                self.items_panel(ui);
                ui.separator();
                egui::ScrollArea::vertical().id_salt("editor").auto_shrink([false, false]).show(ui, |ui| {
                    self.editor(ui);
                    ui.add_space(10.0);
                    self.head_panel(ui);
                });
            });
        });
        egui::CentralPanel::no_frame().show(ui, |ui| self.canvas(ui));

        self.confirm_modal(&ctx);
        self.discard_modal(&ctx);
        self.apply_actions();
        self.settle();
        self.track_laser();
        self.live_reframe();
        self.undo_tick(&ctx);
        self.update_title(&ctx);
        self.screenshot_self_test(&ctx);
        if self.running || self.pending > 0 {
            ctx.request_repaint_after(Duration::from_millis(100));
        } else if self.framing.is_some() {
            ctx.request_repaint_after(Duration::from_millis(300)); // live re-frame
        } else if self.connected.is_none() {
            ctx.request_repaint_after(Duration::from_secs(2));
        }
    }
}

impl Drop for App {
    fn drop(&mut self) {
        // abort a running job, then let the worker switch framing off and exit
        self.control.cancel.store(true, Ordering::SeqCst);
        self.tx = None;
        if let Some(h) = self.worker.take() {
            let _ = h.join();
        }
    }
}

// ----------------------------------------------------------------- widgets

/// Append the system sans-serif face as a glyph fallback (arrows, symbols, accents).
fn install_fallback_font(ctx: &egui::Context) {
    use usvg::fontdb;
    let db = sources::fontdb();
    let query = fontdb::Query { families: &[fontdb::Family::SansSerif], ..Default::default() };
    let Some(data) = db.query(&query).and_then(|id| db.with_face_data(id, |d, _| d.to_vec())) else { return };
    let mut fonts = egui::FontDefinitions::default();
    fonts.font_data.insert("system-sans".into(), Arc::new(egui::FontData::from_owned(data)));
    for family in [egui::FontFamily::Proportional, egui::FontFamily::Monospace] {
        fonts.families.entry(family).or_default().push("system-sans".into());
    }
    ctx.set_fonts(fonts);
}

fn section(ui: &mut egui::Ui, title: &str) {
    ui.add_space(12.0);
    ui.label(RichText::new(title).strong().size(15.0));
    ui.add_space(2.0);
}

fn mm_drag(v: &mut f64, range: std::ops::RangeInclusive<f64>) -> egui::DragValue<'_> {
    egui::DragValue::new(v).speed(0.05).range(range).fixed_decimals(2).suffix(" mm")
}

fn dashed_rect(painter: &egui::Painter, r: egui::Rect, stroke: Stroke) {
    let pts = [r.left_top(), r.right_top(), r.right_bottom(), r.left_bottom(), r.left_top()];
    painter.extend(egui::Shape::dashed_line(&pts, stroke, 5.0, 4.0));
}

fn timing_hint(t: &Timing) -> String {
    format!(
        "Learned from {} lines and batches timed on the machine. Delete {} to start over.",
        t.frames(),
        timing::path().display()
    )
}

fn fmt_dur(d: Duration) -> String {
    let s = d.as_secs();
    if s >= 3600 {
        format!("{}h{:02}m{:02}s", s / 3600, s / 60 % 60, s % 60)
    } else {
        format!("{}m{:02}s", s / 60, s % 60)
    }
}

fn font_picker(ui: &mut egui::Ui, font: &mut String) {
    ui.horizontal(|ui| {
        ui.add(egui::TextEdit::singleline(font).desired_width(170.0));
        ui.menu_button("▾", |ui| {
            egui::ScrollArea::vertical().max_height(320.0).show(ui, |ui| {
                for f in sources::font_families() {
                    if ui.button(f).clicked() {
                        *font = f.clone();
                        ui.close();
                    }
                }
            });
        });
    });
}
