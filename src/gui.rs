//! Native GUI (egui/eframe): mm canvas, operations, properties, machine control.
//!
//! The machine lives on a worker thread that owns the port; the UI sends it
//! commands and reads events. STOP and Pause go through shared atomics, so
//! they act within 50 ms even while the worker waits for an ACK.

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::sync::mpsc::{self, Receiver, Sender};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, anyhow, bail};
use eframe::egui::{self, Color32, Pos2, Sense, Stroke, Vec2};

use crate::config::{self, Config};
use crate::job::{self, GeometryCache, Job, Operation};
use crate::machine::{Aborted, Control, Machine, Progress};
use crate::plan::{Geometry, OpPlan, Plan, Rect};
use crate::protocol::{self as P, MachineSettings, Model};
use crate::sources::{Align, Mode, ShapeKind, Source};
use crate::transport::{self, Port, PortInfo, SerialPort, SimulatedPort};

pub fn run(job_path: Option<PathBuf>) -> Result<()> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("k4laser")
            .with_app_id("k4laser")
            .with_inner_size([1360.0, 860.0])
            .with_min_inner_size([900.0, 600.0]),
        ..Default::default()
    };
    eframe::run_native("k4laser", options, Box::new(move |cc| Ok(Box::new(App::new(cc, job_path)))))
        .map_err(|e| anyhow!("GUI: {e}"))
}

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
            PortChoice::Simulator => "Simulator (no hardware)".into(),
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
            let o = out.clone();
            m.log = Box::new(move |s| o.send(Event::Log(s.to_string())));
            let model = m.connect(Some(&settings), true)?;
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
                    let msg = match &res {
                        Ok(()) => Ok(()),
                        Err(e) if e.is::<Aborted>() => Err("stopped by user — STOP sent".to_string()),
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

// --------------------------------------------------------------------- app

#[derive(Default)]
struct OpView {
    plan: Option<OpPlan>,
    error: Option<String>,
    notes: Vec<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FrameScope {
    Job,
    Selection,
}

struct View {
    /// Screen points per mm.
    zoom: f32,
    /// Screen offset of the work-area origin from the canvas corner.
    pan: Vec2,
    fitted: bool,
}

struct Drag {
    op: usize,
    start_mm: (f64, f64),
    start_pointer: Pos2,
}

struct App {
    job: Job,
    job_path: Option<PathBuf>,
    saved_job: Job,
    selected: Option<usize>,
    cache: GeometryCache,
    compiled_job: Option<Job>,
    views: Vec<OpView>,
    plan: Arc<Plan>,
    plan_fp: u64,
    plan_errors: Vec<String>,
    last_change: Instant,
    textures: HashMap<u64, egui::TextureHandle>,

    cfg: Config,
    ports: Vec<PortInfo>,
    port_choice: Option<PortChoice>,
    tx: Option<Sender<Cmd>>,
    rx: Receiver<Event>,
    worker: Option<JoinHandle<()>>,
    connected: Option<(Model, String)>,
    busy: bool,
    head: Option<(u32, u32)>,
    framing: Option<Rect>,
    pending_frame: Option<(Rect, Option<u64>)>,
    framed_fp: Option<u64>,
    frame_scope: FrameScope,
    live_reframe: bool,
    running: bool,
    control: Arc<Control>,
    progress: Option<Progress>,
    last_result: Option<Result<(), String>>,
    confirm_open: bool,
    confirm_checked: bool,
    jog_step_mm: f64,

    log: Vec<String>,
    view: View,
    drag: Option<Drag>,
    pan_drag: bool,
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
        let mut app = App {
            job: Job::default(),
            job_path: None,
            saved_job: Job::default(),
            selected: None,
            cache: GeometryCache::default(),
            compiled_job: None,
            views: Vec::new(),
            plan: Arc::new(Plan::default()),
            plan_fp: 0,
            plan_errors: Vec::new(),
            last_change: Instant::now(),
            textures: HashMap::new(),
            port_choice: cfg.port.clone().map(PortChoice::Serial),
            cfg,
            ports: transport::list_ports(),
            tx: Some(tx),
            rx,
            worker: Some(worker),
            connected: None,
            busy: false,
            head: None,
            framing: None,
            pending_frame: None,
            framed_fp: None,
            frame_scope: FrameScope::Job,
            live_reframe: true,
            running: false,
            control: Control::new(),
            progress: None,
            last_result: None,
            confirm_open: false,
            confirm_checked: false,
            jog_step_mm: 1.0,
            log,
            view: View { zoom: 8.0, pan: Vec2::splat(40.0), fitted: false },
            drag: None,
            pan_drag: false,
            title: String::new(),
            screenshot: std::env::var_os("K4LASER_SCREENSHOT").map(|p| (PathBuf::from(p), 0)),
        };
        // never auto-pick: CH340 adapters are indistinguishable, and the wrong one may
        // drive something else; the user chooses once, the choice is saved on connect
        if let Some(p) = job_path {
            app.open(&p);
        }
        app
    }

    fn area_px(&self) -> (u32, u32) {
        self.connected.as_ref().and_then(|(m, _)| m.area_px()).unwrap_or(P::K4_AREA_PX)
    }

    fn area_mm(&self) -> (f32, f32) {
        let (w, h) = self.area_px();
        ((w as f64 * P::PITCH_MM) as f32, (h as f64 * P::PITCH_MM) as f32)
    }

    fn logln(&mut self, s: impl Into<String>) {
        self.log.push(s.into());
        if self.log.len() > 2000 {
            self.log.drain(..500);
        }
    }

    fn send(&mut self, cmd: Cmd) {
        if let Some(tx) = &self.tx {
            self.busy = true;
            let _ = tx.send(cmd);
        }
    }

    // ------------------------------------------------------------- files

    fn open(&mut self, path: &Path) {
        match Job::load(path, &Default::default()) {
            Ok(job) => {
                self.saved_job = job.clone();
                self.job = job;
                self.job_path = Some(path.to_path_buf());
                self.selected = None;
                self.view.fitted = false;
                self.logln(format!("opened {}", path.display()));
            }
            Err(e) => self.logln(format!("error: {e:#}")),
        }
    }

    fn save(&mut self, path: Option<PathBuf>) {
        let path = match path.or_else(|| self.job_path.clone()) {
            Some(p) => p,
            None => {
                match rfd::FileDialog::new().add_filter("k4laser job", &["toml"]).set_file_name("job.toml").save_file()
                {
                    Some(p) => p,
                    None => return,
                }
            }
        };
        // relative source paths stay valid only next to the same base directory
        if let Some(dir) = path.parent() {
            let dir = std::path::absolute(dir).unwrap_or_else(|_| dir.to_path_buf());
            if self.job.base_dir.as_os_str().is_empty() {
                self.job.base_dir = dir;
            } else if dir != self.job.base_dir {
                self.logln("note: saved to another directory; relative source paths now resolve from there");
                self.job.base_dir = dir;
            }
        }
        match self.job.save(&path) {
            Ok(()) => {
                self.saved_job = self.job.clone();
                self.job_path = Some(path.clone());
                self.logln(format!("saved {}", path.display()));
            }
            Err(e) => self.logln(format!("error: {e:#}")),
        }
    }

    fn pick_file(&self, name: &str, exts: &[&str]) -> Option<String> {
        let mut d = rfd::FileDialog::new().add_filter(name, exts);
        if !self.job.base_dir.as_os_str().is_empty() {
            d = d.set_directory(&self.job.base_dir);
        }
        let p = d.pick_file()?;
        // store relative to the job directory when possible: jobs stay portable
        Some(match p.strip_prefix(&self.job.base_dir) {
            Ok(rel) if !self.job.base_dir.as_os_str().is_empty() => rel.display().to_string(),
            _ => p.display().to_string(),
        })
    }

    // ----------------------------------------------------------- compile

    fn recompile_if_needed(&mut self) {
        if self.compiled_job.as_ref() == Some(&self.job) {
            return;
        }
        if self.job.base_dir.as_os_str().is_empty() {
            self.job.base_dir = std::env::current_dir().unwrap_or_default();
        }
        let area = self.area_px();
        let mut views = Vec::with_capacity(self.job.ops.len());
        let mut plan = Plan::default();
        let mut errors = Vec::new();
        for op in &self.job.ops {
            let mut v = OpView::default();
            match self.cache.get(op, &self.job.base_dir, &self.job.variables) {
                Err(e) => v.error = Some(format!("{e:#}")),
                Ok(prep) => {
                    v.notes = prep.notes.clone();
                    match job::compile_op(op, &prep) {
                        Err(e) => v.error = Some(format!("{e:#}")),
                        Ok(None) => v.notes.push("nothing to burn".into()),
                        Ok(Some(p)) => {
                            if let Err(e) = job::check_area(&p, area) {
                                v.error = Some(e.to_string());
                            }
                            v.plan = Some(p);
                        }
                    }
                }
            }
            if op.enabled {
                match (&v.error, &v.plan) {
                    (Some(e), _) => errors.push(format!("{}: {e}", op.label())),
                    (None, Some(p)) => plan.ops.push(p.clone()),
                    _ => {}
                }
            }
            views.push(v);
        }
        self.plan_fp = plan.fingerprint();
        self.plan = Arc::new(plan);
        self.plan_errors = errors;
        self.views = views;
        self.compiled_job = Some(self.job.clone());
        self.last_change = Instant::now();
    }

    fn frame_target(&self) -> Option<(Rect, Option<u64>)> {
        match self.frame_scope {
            FrameScope::Job => {
                self.plan.bbox().filter(|_| self.plan_errors.is_empty()).map(|r| (r, Some(self.plan_fp)))
            }
            FrameScope::Selection => {
                let v = self.views.get(self.selected?)?;
                v.plan.as_ref().filter(|_| v.error.is_none()).map(|p| (p.bbox(), None))
            }
        }
    }

    fn start_frame(&mut self) {
        if let Some((rect, fp)) = self.frame_target() {
            self.pending_frame = Some((rect, fp));
            self.send(Cmd::Frame(rect));
        }
    }

    fn stop(&mut self) {
        if self.running {
            self.control.cancel.store(true, Ordering::SeqCst);
            self.logln("STOP requested");
        }
        if self.framing.is_some() || self.pending_frame.is_some() {
            self.pending_frame = None;
            self.send(Cmd::FrameStop);
        }
    }

    fn can_burn(&self) -> bool {
        self.connected.is_some()
            && !self.running
            && !self.plan.ops.is_empty()
            && self.plan_errors.is_empty()
            && self.framed_fp == Some(self.plan_fp)
    }

    // ------------------------------------------------------------ events

    fn drain_events(&mut self) {
        while let Ok(ev) = self.rx.try_recv() {
            match ev {
                Event::Log(s) => self.logln(s),
                Event::Connected { model, port } => {
                    self.logln(format!("connected to {port}: {}", model.describe()));
                    if let Some(PortChoice::Serial(p)) = &self.port_choice
                        && self.cfg.port.as_ref() != Some(p)
                    {
                        self.cfg.port = Some(p.clone());
                        if let Err(e) = config::save(&self.cfg) {
                            self.logln(format!("config not saved: {e:#}"));
                        }
                    }
                    self.connected = Some((model, port));
                    self.compiled_job = None; // the work area may differ
                }
                Event::Disconnected => {
                    self.connected = None;
                    self.head = None;
                    self.framing = None;
                    self.pending_frame = None;
                    self.running = false;
                    self.logln("disconnected");
                }
                Event::State { head, framing } => {
                    self.head = head;
                    self.framing = framing;
                    if let (Some(f), Some((r, fp))) = (framing, self.pending_frame)
                        && f == r
                    {
                        self.pending_frame = None;
                        if fp.is_some() {
                            self.framed_fp = fp;
                        }
                    }
                }
                Event::Idle => self.busy = false,
                Event::Progress(p) => self.progress = Some(p),
                Event::JobDone(res) => {
                    self.running = false;
                    match &res {
                        Ok(()) => self.logln("job finished"),
                        Err(e) => self.logln(format!("job ended: {e}")),
                    }
                    self.last_result = Some(res);
                }
            }
        }
    }

    // ---------------------------------------------------------------- UI

    fn top_bar(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            let idle = !self.running;
            if ui.add_enabled(idle, egui::Button::new("New")).clicked() {
                self.job = Job { base_dir: self.job.base_dir.clone(), ..Default::default() };
                self.saved_job = self.job.clone();
                self.job_path = None;
                self.selected = None;
            }
            if ui.add_enabled(idle, egui::Button::new("Open…")).clicked()
                && let Some(p) = rfd::FileDialog::new().add_filter("k4laser job", &["toml"]).pick_file()
            {
                self.open(&p);
            }
            if ui.button("Save").clicked() {
                self.save(None);
            }
            if ui.button("Save as…").clicked()
                && let Some(p) =
                    rfd::FileDialog::new().add_filter("k4laser job", &["toml"]).set_file_name("job.toml").save_file()
            {
                self.save(Some(p));
            }
            if ui.button("Reload sources").on_hover_text("Re-read image/SVG/DXF files and fonts").clicked() {
                self.cache = GeometryCache::default();
                self.compiled_job = None;
            }
            ui.separator();

            let connected = self.connected.is_some();
            ui.add_enabled_ui(!connected && !self.busy, |ui| {
                let text = self.port_choice.as_ref().map(PortChoice::label).unwrap_or_else(|| "select port…".into());
                egui::ComboBox::from_id_salt("port").width(360.0).selected_text(text).show_ui(ui, |ui| {
                    for p in &self.ports {
                        let choice = PortChoice::Serial(p.stable_path().to_path_buf());
                        let label = format!("{}\n    {}", p.stable_path().display(), p.label());
                        let text = if p.holders.is_empty() {
                            egui::RichText::new(label)
                        } else {
                            egui::RichText::new(label).color(Color32::from_rgb(220, 120, 40))
                        };
                        ui.selectable_value(&mut self.port_choice, Some(choice), text);
                    }
                    ui.selectable_value(
                        &mut self.port_choice,
                        Some(PortChoice::Simulator),
                        PortChoice::Simulator.label(),
                    );
                });
                if ui.button("Rescan").on_hover_text("Rescan serial ports").clicked() {
                    self.ports = transport::list_ports();
                }
            });
            if connected {
                if ui.add_enabled(!self.running, egui::Button::new("Disconnect")).clicked() {
                    self.send(Cmd::Disconnect);
                }
            } else if ui.add_enabled(self.port_choice.is_some() && !self.busy, egui::Button::new("Connect")).clicked() {
                let choice = self.port_choice.clone().unwrap();
                self.logln(format!("connecting to {} (handshake, settings, home)…", choice.label()));
                self.send(Cmd::Connect(choice, self.cfg.settings));
            }
            match &self.connected {
                Some((m, _)) => ui.label(
                    egui::RichText::new(format!("connected: {}", m.describe())).color(Color32::from_rgb(60, 170, 90)),
                ),
                None => ui.label(egui::RichText::new("not connected").weak()),
            };
            if self.busy {
                ui.spinner();
            }
        });
    }

    fn ops_panel(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.heading("Operations");
            ui.menu_button("+ Add", |ui| {
                for kind in Source::KINDS {
                    if ui.button(kind).clicked() {
                        let mut op = Operation::new(Source::default_of(kind).unwrap());
                        op.x_mm = 5.0;
                        op.y_mm = 5.0;
                        self.job.ops.push(op);
                        self.selected = Some(self.job.ops.len() - 1);
                        ui.close();
                    }
                }
            });
        });
        ui.separator();
        let mut action: Option<(usize, &str)> = None;
        egui::ScrollArea::vertical().id_salt("ops").max_height(ui.available_height() * 0.6).show(ui, |ui| {
            for i in 0..self.job.ops.len() {
                let err = self.views.get(i).is_some_and(|v| v.error.is_some());
                ui.horizontal(|ui| {
                    ui.checkbox(&mut self.job.ops[i].enabled, "");
                    let op = &self.job.ops[i];
                    let mut text = egui::RichText::new(format!("{}. {} [{}]", i + 1, op.label(), op.mode.as_str()));
                    if err {
                        text = text.color(Color32::from_rgb(220, 60, 60));
                    } else if !op.enabled {
                        text = text.weak();
                    }
                    if ui.selectable_label(self.selected == Some(i), text).clicked() {
                        self.selected = Some(i);
                    }
                });
            }
        });
        ui.horizontal(|ui| {
            let sel = self.selected.filter(|&i| i < self.job.ops.len());
            ui.add_enabled_ui(sel.is_some(), |ui| {
                if ui.button("▲").on_hover_text("Move up").clicked() {
                    action = sel.map(|i| (i, "up"));
                }
                if ui.button("▼").on_hover_text("Move down").clicked() {
                    action = sel.map(|i| (i, "down"));
                }
                if ui.button("Duplicate").clicked() {
                    action = sel.map(|i| (i, "dup"));
                }
                if ui.button("Delete").clicked() {
                    action = sel.map(|i| (i, "del"));
                }
            });
        });
        if let Some((i, a)) = action {
            let n = self.job.ops.len();
            match a {
                "up" if i > 0 => {
                    self.job.ops.swap(i, i - 1);
                    self.selected = Some(i - 1);
                }
                "down" if i + 1 < n => {
                    self.job.ops.swap(i, i + 1);
                    self.selected = Some(i + 1);
                }
                "dup" => {
                    let mut op = self.job.ops[i].clone();
                    op.x_mm += 2.0;
                    op.y_mm += 2.0;
                    self.job.ops.insert(i + 1, op);
                    self.selected = Some(i + 1);
                }
                "del" => {
                    self.job.ops.remove(i);
                    self.selected = if self.job.ops.is_empty() { None } else { Some(i.min(self.job.ops.len() - 1)) };
                }
                _ => {}
            }
        }

        ui.add_space(8.0);
        egui::CollapsingHeader::new("Variables  ${name}").default_open(!self.job.variables.is_empty()).show(ui, |ui| {
            let mut remove = None;
            let mut rename: Option<(String, String)> = None;
            // plain rows: a Grid cell never grows past its first-frame width
            for (k, v) in self.job.variables.iter_mut() {
                ui.horizontal(|ui| {
                    let mut key = k.clone();
                    if ui.add(egui::TextEdit::singleline(&mut key).desired_width(70.0)).changed() {
                        rename = Some((k.clone(), key));
                    }
                    ui.add(egui::TextEdit::singleline(v).desired_width(ui.available_width() - 32.0));
                    if ui.small_button("×").clicked() {
                        remove = Some(k.clone());
                    }
                });
            }
            if let Some(k) = remove {
                self.job.variables.remove(&k);
            }
            if let Some((old, new)) = rename
                && !new.is_empty()
                && !self.job.variables.contains_key(&new)
            {
                let v = self.job.variables.remove(&old).unwrap_or_default();
                self.job.variables.insert(new, v);
            }
            if ui.button("+ variable").clicked() {
                let mut n = 1;
                while self.job.variables.contains_key(&format!("var{n}")) {
                    n += 1;
                }
                self.job.variables.insert(format!("var{n}"), String::new());
            }
        });

        ui.add_space(8.0);
        egui::CollapsingHeader::new("Job summary").default_open(true).show(ui, |ui| {
            let p = &self.plan;
            match p.bbox() {
                Some(b) => {
                    ui.label(format!("{} operation(s), {} data frames", p.ops.len(), p.data_frames()));
                    ui.label(format!("bounding box {}", b.describe_mm()));
                }
                None => {
                    ui.label("nothing to burn");
                }
            }
            for e in &self.plan_errors {
                ui.colored_label(Color32::from_rgb(220, 60, 60), e);
            }
        });
    }

    fn properties_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("Properties");
        ui.separator();
        let Some(i) = self.selected.filter(|&i| i < self.job.ops.len()) else {
            ui.label("Select an operation, or add one with + Add.");
            ui.add_space(12.0);
            ui.label(egui::RichText::new("Canvas: drag to move (snaps to 0.05 mm, Ctrl: 1 mm), wheel to zoom, right-drag to pan, double-click to fit. Arrows nudge 0.05 mm (Shift: 1 mm). Esc = STOP.").weak());
            return;
        };
        let base_dir = self.job.base_dir.clone();
        let mut picked: Option<String> = None;
        let want_pick: Option<(&str, &[&str])> = {
            let op = &mut self.job.ops[i];
            egui::Grid::new("op").num_columns(2).spacing([8.0, 6.0]).show(ui, |ui| {
                ui.label("Name");
                ui.text_edit_singleline(&mut op.name);
                ui.end_row();
                ui.label("Mode");
                ui.horizontal(|ui| {
                    for &m in op.source.modes() {
                        ui.selectable_value(&mut op.mode, m, m.as_str());
                    }
                });
                ui.end_row();
                ui.label("X");
                ui.add(mm_value(&mut op.x_mm, 0.0..=200.0));
                ui.end_row();
                ui.label("Y");
                ui.add(mm_value(&mut op.y_mm, 0.0..=200.0));
                ui.end_row();
                ui.label("Power");
                ui.add(egui::Slider::new(&mut op.power, 0.0..=100.0).step_by(0.1).fixed_decimals(1).suffix(" %"));
                ui.end_row();
                ui.label("Depth").on_hover_text(
                    "Vendor \"carving depth\": dwell per pixel, unit unknown (0-100). Calibrate with a test grid.",
                );
                ui.add(egui::Slider::new(&mut op.depth, 0..=P::DEPTH_MAX));
                ui.end_row();
                ui.label("Passes");
                ui.add(egui::DragValue::new(&mut op.passes).range(1..=50));
                ui.end_row();
            });
            ui.separator();
            ui.label(egui::RichText::new(format!("Source: {}", op.source.kind())).strong());
            source_editor(ui, &mut op.source)
        };
        if let Some((name, exts)) = want_pick {
            picked = self.pick_file(name, exts);
        }
        if let Some(p) = picked {
            match &mut self.job.ops[i].source {
                Source::Image(s) => s.path = p,
                Source::Svg(s) => s.path = p,
                Source::Dxf(s) => s.path = p,
                _ => {}
            }
        }
        // keep mode valid when the source type changes
        let op = &mut self.job.ops[i];
        if !op.source.modes().contains(&op.mode) {
            op.mode = op.source.modes()[0];
        }

        ui.separator();
        if let Some(v) = self.views.get(i) {
            if let Some(p) = &v.plan {
                ui.label(egui::RichText::new(p.bbox().describe_mm()).monospace());
                let detail = match &*p.geometry {
                    Geometry::Raster(bm) => format!("raster: {} rows, {} burn px", p.frame_count(), bm.count()),
                    Geometry::Vector(segs) => {
                        let (burn, travel) = p.lengths_mm();
                        format!("vector: {} segments, burn {burn:.1} mm, travel {travel:.1} mm", segs.len())
                    }
                };
                ui.label(detail);
            }
            if let Some(e) = &v.error {
                ui.colored_label(Color32::from_rgb(220, 60, 60), e);
            }
            for n in &v.notes {
                ui.label(egui::RichText::new(format!("• {n}")).weak());
            }
        }
        if base_dir.as_os_str().is_empty() {
            ui.label(
                egui::RichText::new("Relative paths resolve from the current directory until the job is saved.").weak(),
            );
        }
    }

    fn machine_panel(&mut self, ui: &mut egui::Ui) {
        let connected = self.connected.is_some();
        let idle = connected && !self.running && !self.busy;
        ui.horizontal(|ui| {
            // ---- STOP, always first and largest
            let stop = egui::Button::new(egui::RichText::new("STOP").strong().size(20.0).color(Color32::WHITE))
                .fill(Color32::from_rgb(200, 30, 30))
                .min_size(Vec2::new(110.0, 44.0));
            if ui.add_enabled(connected, stop).on_hover_text("Esc — aborts the job (sends 0x16) and switches framing off").clicked() {
                self.stop();
            }
            ui.separator();

            // ---- position
            ui.vertical(|ui| {
                ui.horizontal(|ui| {
                    if ui.add_enabled(idle, egui::Button::new("Home")).clicked() {
                        self.send(Cmd::Home);
                    }
                    let head = match self.head {
                        Some((x, y)) => format!("head {:.2}, {:.2} mm", x as f64 * P::PITCH_MM, y as f64 * P::PITCH_MM),
                        None => "head position unknown".into(),
                    };
                    ui.label(egui::RichText::new(head).monospace());
                });
                ui.horizontal(|ui| {
                    let step = (self.jog_step_mm / P::PITCH_MM).round() as i64;
                    ui.add_enabled_ui(idle, |ui| {
                        if ui.button("←").clicked() {
                            self.send(Cmd::Jog(-step, 0));
                        }
                        if ui.button("↑").clicked() {
                            self.send(Cmd::Jog(0, -step));
                        }
                        if ui.button("↓").clicked() {
                            self.send(Cmd::Jog(0, step));
                        }
                        if ui.button("→").clicked() {
                            self.send(Cmd::Jog(step, 0));
                        }
                    });
                    egui::ComboBox::from_id_salt("jogstep").width(70.0).selected_text(format!("{} mm", self.jog_step_mm)).show_ui(ui, |ui| {
                        for s in [0.05, 0.1, 0.5, 1.0, 5.0, 10.0] {
                            ui.selectable_value(&mut self.jog_step_mm, s, format!("{s} mm"));
                        }
                    });
                });
            });
            ui.separator();

            // ---- framing
            ui.vertical(|ui| {
                ui.horizontal(|ui| {
                    let framing = self.framing.is_some() || self.pending_frame.is_some();
                    let label = if framing { "Stop framing" } else { "Frame" };
                    let target = self.frame_target();
                    let btn = egui::Button::new(egui::RichText::new(label).strong()).min_size(Vec2::new(120.0, 26.0));
                    let enabled = connected && !self.running && (framing || target.is_some());
                    if ui.add_enabled(enabled, btn).on_hover_text("Trace the bounding box with the low-power beam to align the piece").clicked() {
                        if framing {
                            self.pending_frame = None;
                            self.send(Cmd::FrameStop);
                        } else {
                            self.start_frame();
                        }
                    }
                    ui.selectable_value(&mut self.frame_scope, FrameScope::Job, "job");
                    ui.selectable_value(&mut self.frame_scope, FrameScope::Selection, "selection");
                });
                ui.checkbox(&mut self.live_reframe, "follow edits while framing");
                if let Some(r) = self.framing {
                    ui.label(egui::RichText::new(format!("framing {}", r.describe_mm())).color(Color32::from_rgb(230, 80, 60)).monospace());
                }
            });
            ui.separator();

            // ---- burn
            ui.vertical(|ui| {
                ui.horizontal(|ui| {
                    let burn = egui::Button::new(egui::RichText::new("Burn…").strong()).min_size(Vec2::new(100.0, 26.0));
                    let hint = if !connected {
                        "connect first"
                    } else if self.plan.ops.is_empty() {
                        "nothing to burn"
                    } else if !self.plan_errors.is_empty() {
                        "fix the errors listed in the job summary"
                    } else if self.framed_fp != Some(self.plan_fp) {
                        "frame the job (scope: job) first — the interlock requires the current geometry to have been traced"
                    } else {
                        "confirm and start"
                    };
                    if ui.add_enabled(self.can_burn(), burn).on_hover_text(hint).on_disabled_hover_text(hint).clicked() {
                        self.confirm_open = true;
                        self.confirm_checked = false;
                    }
                    let paused = self.control.paused.load(Ordering::SeqCst);
                    if ui.add_enabled(self.running, egui::Button::new(if paused { "Resume" } else { "Pause" })).on_hover_text("Holds the next row/batch on the host side").clicked() {
                        self.control.paused.store(!paused, Ordering::SeqCst);
                    }
                    if !self.running && connected && self.framed_fp != Some(self.plan_fp) && !self.plan.ops.is_empty() {
                        ui.label(egui::RichText::new("frame first").weak());
                    }
                });
                if let Some(p) = &self.progress {
                    let frac = p.step as f32 / p.total.max(1) as f32;
                    let eta = p.eta.map(|e| format!(", ETA {}", fmt_dur(e))).unwrap_or_default();
                    ui.add(egui::ProgressBar::new(frac).desired_width(360.0).text(format!("{}/{}  {}{eta}", p.step, p.total, fmt_dur(p.elapsed))));
                    ui.label(egui::RichText::new(&p.label).weak());
                }
                match &self.last_result {
                    Some(Ok(())) if !self.running => {
                        ui.colored_label(Color32::from_rgb(60, 170, 90), "last job completed");
                    }
                    Some(Err(e)) if !self.running => {
                        ui.colored_label(Color32::from_rgb(220, 60, 60), e);
                    }
                    _ => {}
                }
            });
            ui.separator();

            egui::CollapsingHeader::new("Experimental").default_open(false).show(ui, |ui| {
                ui.label(egui::RichText::new("Hidden V2.7 commands; effect unconfirmed\n(suspected accessory/fan output).").weak());
                ui.horizontal(|ui| {
                    if ui.add_enabled(idle, egui::Button::new("0xFD")).clicked() {
                        self.send(Cmd::Aux(true));
                    }
                    if ui.add_enabled(idle, egui::Button::new("0xFE")).clicked() {
                        self.send(Cmd::Aux(false));
                    }
                });
            });
        });
    }

    fn log_panel(&mut self, ui: &mut egui::Ui) {
        egui::ScrollArea::vertical().id_salt("log").stick_to_bottom(true).show(ui, |ui| {
            for line in self.log.iter().rev().take(300).rev() {
                let mut t = egui::RichText::new(line).monospace().size(11.0);
                if line.starts_with("error") || line.starts_with("job ended") {
                    t = t.color(Color32::from_rgb(220, 60, 60));
                }
                ui.label(t);
            }
        });
    }

    fn confirm_modal(&mut self, ctx: &egui::Context) {
        if !self.confirm_open {
            return;
        }
        let modal = egui::Modal::new(egui::Id::unique("confirm-burn")).show(ctx, |ui| {
            ui.set_width(460.0);
            ui.heading("Start burning?");
            ui.add_space(6.0);
            let p = &self.plan;
            if let Some(b) = p.bbox() {
                ui.label(format!("{} operation(s) in {}", p.ops.len(), b.describe_mm()));
            }
            for line in p.summary() {
                ui.label(egui::RichText::new(line).monospace().size(11.0));
            }
            ui.add_space(8.0);
            ui.checkbox(&mut self.confirm_checked, "Eye protection on, piece secured and framed, extraction suitable for the material, nothing flammable nearby.");
            ui.add_space(8.0);
            ui.horizontal(|ui| {
                let go = egui::Button::new(egui::RichText::new("Burn").strong().color(Color32::WHITE)).fill(Color32::from_rgb(200, 90, 20));
                if ui.add_enabled(self.confirm_checked && self.can_burn(), go).clicked() {
                    self.confirm_open = false;
                    self.control = Control::new();
                    self.running = true;
                    self.progress = None;
                    self.last_result = None;
                    self.pending_frame = None;
                    let plan = self.plan.clone();
                    self.logln(format!("burn started: {} operation(s)", plan.ops.len()));
                    self.send(Cmd::Run(plan, self.control.clone()));
                }
                if ui.button("Cancel").clicked() {
                    self.confirm_open = false;
                }
            });
        });
        if modal.should_close() {
            self.confirm_open = false;
        }
    }

    // ------------------------------------------------------------ canvas

    fn to_screen(&self, origin: Pos2, x_mm: f32, y_mm: f32) -> Pos2 {
        origin + self.view.pan + Vec2::new(x_mm, y_mm) * self.view.zoom
    }

    fn canvas(&mut self, ui: &mut egui::Ui) {
        let (resp, painter) = ui.allocate_painter(ui.available_size(), Sense::click_and_drag());
        let rect = resp.rect;
        let (aw, ah) = self.area_mm();
        if !self.view.fitted || resp.double_clicked() {
            let zoom = ((rect.width() - 70.0) / aw).min((rect.height() - 60.0) / ah).max(1.0);
            self.view.zoom = zoom;
            self.view.pan =
                Vec2::new((rect.width() - aw * zoom) / 2.0 + 10.0, (rect.height() - ah * zoom) / 2.0 + 10.0);
            self.view.fitted = true;
        }
        let o = rect.min;
        let dark = ui.visuals().dark_mode;
        let (bg, area_fill, grid_minor, grid_major, ink) = if dark {
            (
                Color32::from_gray(24),
                Color32::from_gray(40),
                Color32::from_gray(52),
                Color32::from_gray(75),
                Color32::from_rgb(255, 170, 60),
            )
        } else {
            (
                Color32::from_gray(225),
                Color32::WHITE,
                Color32::from_gray(236),
                Color32::from_gray(200),
                Color32::from_gray(20),
            )
        };
        painter.rect_filled(rect, 0.0, bg);
        let area = egui::Rect::from_min_max(self.to_screen(o, 0.0, 0.0), self.to_screen(o, aw, ah));
        painter.rect_filled(area, 0.0, area_fill);
        let painter = painter.with_clip_rect(rect);

        // grid: 1 mm (when legible), 5 mm, 10 mm with labels
        let z = self.view.zoom;
        let text_col = ui.visuals().weak_text_color();
        let font = egui::FontId::monospace(10.0);
        let mut mm = 0;
        while mm as f32 <= aw {
            let x = area.left() + mm as f32 * z;
            let (show, col) = if mm % 10 == 0 {
                (true, grid_major)
            } else if mm % 5 == 0 {
                (z >= 3.0, grid_minor)
            } else {
                (z >= 7.0, grid_minor)
            };
            if show {
                painter.line_segment([Pos2::new(x, area.top()), Pos2::new(x, area.bottom())], Stroke::new(1.0, col));
            }
            if mm % 10 == 0 {
                painter.text(Pos2::new(x, area.top() - 3.0), egui::Align2::CENTER_BOTTOM, mm, font.clone(), text_col);
            }
            mm += 1;
        }
        let mut mm = 0;
        while mm as f32 <= ah {
            let y = area.top() + mm as f32 * z;
            let (show, col) = if mm % 10 == 0 {
                (true, grid_major)
            } else if mm % 5 == 0 {
                (z >= 3.0, grid_minor)
            } else {
                (z >= 7.0, grid_minor)
            };
            if show {
                painter.line_segment([Pos2::new(area.left(), y), Pos2::new(area.right(), y)], Stroke::new(1.0, col));
            }
            if mm % 10 == 0 {
                painter.text(Pos2::new(area.left() - 4.0, y), egui::Align2::RIGHT_CENTER, mm, font.clone(), text_col);
            }
            mm += 1;
        }
        painter.rect_stroke(area, 0.0, Stroke::new(1.5, grid_major), egui::StrokeKind::Outside);

        // operations
        let px = P::PITCH_MM as f32;
        let mut used_textures = Vec::new();
        for (i, (op, v)) in self.job.ops.iter().zip(&self.views).enumerate() {
            let Some(p) = &v.plan else { continue };
            let tint = if !op.enabled {
                ink.gamma_multiply(0.25)
            } else if v.error.is_some() {
                Color32::from_rgb(220, 60, 60)
            } else {
                ink
            };
            let r = egui::Rect::from_min_size(
                self.to_screen(o, p.x as f32 * px, p.y as f32 * px),
                Vec2::new(p.width as f32 * px * z, p.height as f32 * px * z),
            );
            match &*p.geometry {
                Geometry::Raster(bm) => {
                    if bm.width <= 8192 && bm.height <= 8192 {
                        let tex = self.textures.entry(p.geom_hash).or_insert_with(|| {
                            let pixels = bm
                                .data
                                .iter()
                                .map(|&b| if b { Color32::WHITE } else { Color32::TRANSPARENT })
                                .collect();
                            let img = egui::ColorImage::new([bm.width, bm.height], pixels);
                            ui.ctx().load_texture(format!("op-{:x}", p.geom_hash), img, egui::TextureOptions::NEAREST)
                        });
                        used_textures.push(p.geom_hash);
                        painter.image(tex.id(), r, egui::Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)), tint);
                    }
                }
                Geometry::Vector(segs) => {
                    let stroke = Stroke::new(1.2, tint);
                    let base = (p.x as f32 * px, p.y as f32 * px);
                    for s in segs {
                        let a = self.to_screen(o, base.0 + s[0] as f32 * px, base.1 + s[1] as f32 * px);
                        let b = self.to_screen(o, base.0 + s[2] as f32 * px, base.1 + s[3] as f32 * px);
                        painter.line_segment([a, b], stroke);
                    }
                }
            }
            if self.selected == Some(i) {
                let sel = ui.visuals().selection.stroke.color;
                painter.rect_stroke(r.expand(2.0), 0.0, Stroke::new(1.5, sel), egui::StrokeKind::Outside);
                painter.text(
                    r.left_top() + Vec2::new(0.0, -4.0),
                    egui::Align2::LEFT_BOTTOM,
                    op.label(),
                    egui::FontId::proportional(11.0),
                    sel,
                );
            }
        }
        self.textures.retain(|k, _| used_textures.contains(k));

        // job bounding box (what "Frame: job" traces)
        if let Some(b) = self.plan.bbox() {
            let r = egui::Rect::from_min_max(
                self.to_screen(o, b.x as f32 * px, b.y as f32 * px),
                self.to_screen(o, (b.x + b.w) as f32 * px, (b.y + b.h) as f32 * px),
            );
            let pts = [r.left_top(), r.right_top(), r.right_bottom(), r.left_bottom(), r.left_top()];
            painter.extend(egui::Shape::dashed_line(&pts, Stroke::new(1.0, Color32::from_rgb(80, 160, 255)), 6.0, 4.0));
        }
        // live framing rectangle
        if let Some(f) = self.framing {
            let r = egui::Rect::from_min_max(
                self.to_screen(o, f.x as f32 * px, f.y as f32 * px),
                self.to_screen(o, (f.x + f.w) as f32 * px, (f.y + f.h) as f32 * px),
            );
            painter.rect_stroke(r, 0.0, Stroke::new(2.5, Color32::from_rgb(230, 50, 40)), egui::StrokeKind::Middle);
        }
        // head
        if let Some((hx, hy)) = self.head {
            let c = self.to_screen(o, hx as f32 * px, hy as f32 * px);
            let s = Stroke::new(1.5, Color32::from_rgb(40, 140, 230));
            painter.line_segment([c - Vec2::new(8.0, 0.0), c + Vec2::new(8.0, 0.0)], s);
            painter.line_segment([c - Vec2::new(0.0, 8.0), c + Vec2::new(0.0, 8.0)], s);
            painter.circle_stroke(c, 4.0, s);
        }

        // ---- interaction
        let pointer = resp.hover_pos();
        let to_mm = |p: Pos2, view: &View| {
            let v = (p - o - view.pan) / view.zoom;
            (v.x as f64, v.y as f64)
        };
        if let Some(pos) = pointer {
            let (x, y) = to_mm(pos, &self.view);
            painter.text(
                rect.left_bottom() + Vec2::new(8.0, -6.0),
                egui::Align2::LEFT_BOTTOM,
                format!("x {x:7.2} mm   y {y:7.2} mm"),
                egui::FontId::monospace(12.0),
                ui.visuals().text_color(),
            );
            let scroll = ui.ctx().input(|i| i.smooth_scroll_delta.y);
            if scroll != 0.0 {
                let factor = (scroll / 300.0).exp();
                let new_zoom = (self.view.zoom * factor).clamp(1.0, 400.0);
                let anchor = pos - o - self.view.pan;
                self.view.pan += anchor - anchor * (new_zoom / self.view.zoom);
                self.view.zoom = new_zoom;
            }
        }
        let editable = !self.running;
        let hit = |app: &App, pos: Pos2| -> Option<usize> {
            let (x, y) = to_mm(pos, &app.view);
            let (x, y) = ((x / P::PITCH_MM) as i64, (y / P::PITCH_MM) as i64);
            let slop = (4.0 / app.view.zoom as f64 / P::PITCH_MM) as i64;
            app.views.iter().enumerate().rev().find_map(|(i, v)| {
                let p = v.plan.as_ref()?;
                let inside = x >= p.x as i64 - slop
                    && x <= (p.x + p.width) as i64 + slop
                    && y >= p.y as i64 - slop
                    && y <= (p.y + p.height) as i64 + slop;
                inside.then_some(i)
            })
        };
        if resp.clicked()
            && let Some(pos) = resp.interact_pointer_pos()
        {
            self.selected = hit(self, pos);
        }
        if resp.drag_started_by(egui::PointerButton::Primary)
            && editable
            && let Some(pos) = resp.interact_pointer_pos()
            && let Some(i) = hit(self, pos)
        {
            self.selected = Some(i);
            let op = &self.job.ops[i];
            self.drag = Some(Drag { op: i, start_mm: (op.x_mm, op.y_mm), start_pointer: pos });
        }
        if resp.drag_started_by(egui::PointerButton::Secondary) || resp.drag_started_by(egui::PointerButton::Middle) {
            self.pan_drag = true;
        }
        if self.pan_drag {
            self.view.pan += resp.drag_delta();
        }
        if let (Some(d), Some(pos)) = (&self.drag, resp.interact_pointer_pos())
            && d.op < self.job.ops.len()
        {
            let delta = (pos - d.start_pointer) / self.view.zoom;
            let snap = if ui.ctx().input(|i| i.modifiers.ctrl) { 1.0 } else { P::PITCH_MM };
            let q = |v: f64| ((v / snap).round() * snap * 1e6).round() / 1e6;
            let op = &mut self.job.ops[d.op];
            op.x_mm = q((d.start_mm.0 + delta.x as f64).max(0.0));
            op.y_mm = q((d.start_mm.1 + delta.y as f64).max(0.0));
        }
        if resp.drag_stopped() {
            self.drag = None;
            self.pan_drag = false;
        }
    }

    fn keyboard(&mut self, ctx: &egui::Context) {
        if ctx.input(|i| i.key_pressed(egui::Key::Escape))
            && (self.running || self.framing.is_some() || self.pending_frame.is_some())
        {
            self.stop();
        }
        if ctx.egui_wants_keyboard_input() || self.running {
            return;
        }
        let Some(i) = self.selected.filter(|&i| i < self.job.ops.len()) else { return };
        let (dx, dy, shift) = ctx.input(|inp| {
            let mut d = (0.0, 0.0);
            if inp.key_pressed(egui::Key::ArrowLeft) {
                d.0 -= 1.0;
            }
            if inp.key_pressed(egui::Key::ArrowRight) {
                d.0 += 1.0;
            }
            if inp.key_pressed(egui::Key::ArrowUp) {
                d.1 -= 1.0;
            }
            if inp.key_pressed(egui::Key::ArrowDown) {
                d.1 += 1.0;
            }
            (d.0, d.1, inp.modifiers.shift)
        });
        if dx != 0.0 || dy != 0.0 {
            let step = if shift { 1.0 } else { P::PITCH_MM };
            let op = &mut self.job.ops[i];
            op.x_mm = (((op.x_mm + dx * step).max(0.0)) * 1e6).round() / 1e6;
            op.y_mm = (((op.y_mm + dy * step).max(0.0)) * 1e6).round() / 1e6;
        }
        if ctx.input(|i| i.key_pressed(egui::Key::Delete)) {
            self.job.ops.remove(i);
            self.selected = None;
        }
    }

    fn live_reframe(&mut self) {
        if !self.live_reframe || self.busy || self.running || self.pending_frame.is_some() {
            return;
        }
        let Some(current) = self.framing else { return };
        let Some((rect, fp)) = self.frame_target() else { return };
        if rect == current {
            // same rectangle already traced: the interlock is satisfied
            if fp.is_some() {
                self.framed_fp = fp;
            }
        } else if self.last_change.elapsed() > Duration::from_millis(250) {
            self.start_frame();
        }
    }

    fn update_title(&mut self, ctx: &egui::Context) {
        let name = self
            .job_path
            .as_ref()
            .and_then(|p| p.file_name())
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_else(|| "untitled".into());
        let dirty = if self.job != self.saved_job { " •" } else { "" };
        let title = format!("{name}{dirty} — k4laser");
        if title != self.title {
            ctx.send_viewport_cmd(egui::ViewportCommand::Title(title.clone()));
            self.title = title;
        }
    }

    /// `K4LASER_SCREENSHOT=out.png k4laser gui job.toml`: render, save a screenshot, quit.
    fn screenshot_self_test(&mut self, ctx: &egui::Context) {
        let Some((_, frames)) = self.screenshot.as_mut() else { return };
        *frames += 1;
        let n = *frames;
        // K4LASER_SELFTEST_SIM=1: also connect to the simulator and frame the job
        if std::env::var_os("K4LASER_SELFTEST_SIM").is_some() {
            if n == 2 {
                self.port_choice = Some(PortChoice::Simulator);
                self.send(Cmd::Connect(PortChoice::Simulator, self.cfg.settings));
            }
            if n < 25
                && self.connected.is_some()
                && !self.busy
                && self.framing.is_none()
                && self.pending_frame.is_none()
            {
                self.selected = Some(1.min(self.job.ops.len().saturating_sub(1)));
                self.start_frame();
            }
        }
        let Some((path, frames)) = self.screenshot.as_mut() else { return };
        if *frames == 30 {
            let (path, ctx2) = (path.clone(), ctx.clone());
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
        if *frames <= 30 {
            ctx.request_repaint();
        }
    }
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        self.drain_events();
        self.recompile_if_needed();
        self.keyboard(&ctx);

        egui::Panel::top("top").show(ui, |ui| {
            ui.add_space(4.0);
            self.top_bar(ui);
            ui.add_space(2.0);
        });
        egui::Panel::bottom("machine").show(ui, |ui| {
            ui.add_space(6.0);
            self.machine_panel(ui);
            ui.add_space(4.0);
        });
        egui::Panel::bottom("log").resizable(true).default_size(110.0).show(ui, |ui| self.log_panel(ui));
        egui::Panel::left("ops").resizable(true).default_size(290.0).show(ui, |ui| {
            ui.add_enabled_ui(!self.running, |ui| self.ops_panel(ui));
        });
        egui::Panel::right("props").resizable(true).default_size(330.0).show(ui, |ui| {
            egui::ScrollArea::vertical().id_salt("props").show(ui, |ui| {
                ui.add_enabled_ui(!self.running, |ui| self.properties_panel(ui));
            });
        });
        egui::CentralPanel::no_frame().show(ui, |ui| self.canvas(ui));

        self.confirm_modal(&ctx);
        self.recompile_if_needed();
        self.live_reframe();
        self.update_title(&ctx);
        self.screenshot_self_test(&ctx);
        if self.running || self.busy {
            ctx.request_repaint_after(Duration::from_millis(100));
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
    let db = crate::sources::fontdb();
    let query = fontdb::Query { families: &[fontdb::Family::SansSerif], ..Default::default() };
    let Some(data) = db.query(&query).and_then(|id| db.with_face_data(id, |d, _| d.to_vec())) else { return };
    let mut fonts = egui::FontDefinitions::default();
    fonts.font_data.insert("system-sans".into(), Arc::new(egui::FontData::from_owned(data)));
    for family in [egui::FontFamily::Proportional, egui::FontFamily::Monospace] {
        fonts.families.entry(family).or_default().push("system-sans".into());
    }
    ctx.set_fonts(fonts);
}

fn mm_value(v: &mut f64, range: std::ops::RangeInclusive<f64>) -> egui::DragValue<'_> {
    egui::DragValue::new(v).speed(P::PITCH_MM).range(range).fixed_decimals(2).suffix(" mm")
}

fn fmt_dur(d: Duration) -> String {
    let s = d.as_secs();
    if s >= 3600 {
        format!("{}h{:02}m{:02}s", s / 3600, s / 60 % 60, s % 60)
    } else {
        format!("{}m{:02}s", s / 60, s % 60)
    }
}

/// Editor for the source fields; returns a file-picker request.
fn source_editor(ui: &mut egui::Ui, src: &mut Source) -> Option<(&'static str, &'static [&'static str])> {
    let mut pick = None;
    egui::Grid::new("src").num_columns(2).spacing([8.0, 6.0]).show(ui, |ui| match src {
        Source::Text(t) => {
            ui.label("Text");
            ui.add(egui::TextEdit::multiline(&mut t.text).desired_rows(2).desired_width(200.0));
            ui.end_row();
            ui.label("Font").on_hover_text("Family, optionally followed by Bold / Italic / Light…");
            font_picker(ui, &mut t.font);
            ui.end_row();
            ui.label("Size (em)");
            ui.add(mm_value(&mut t.size_mm, 0.3..=76.0));
            ui.end_row();
            ui.label("Align");
            ui.horizontal(|ui| {
                ui.selectable_value(&mut t.align, Align::Left, "left");
                ui.selectable_value(&mut t.align, Align::Center, "center");
                ui.selectable_value(&mut t.align, Align::Right, "right");
            });
            ui.end_row();
        }
        Source::Image(s) => {
            path_row(
                ui,
                &mut s.path,
                &mut pick,
                ("Images", &["png", "jpg", "jpeg", "bmp", "gif", "webp", "tif", "tiff"]),
            );
            ui.label("Width");
            ui.add(mm_value(&mut s.width_mm, 0.0..=200.0)).on_hover_text("0 = from height (or dpi)");
            ui.end_row();
            ui.label("Height");
            ui.add(mm_value(&mut s.height_mm, 0.0..=200.0)).on_hover_text("0 = keep aspect ratio");
            ui.end_row();
            ui.label("DPI");
            ui.add(egui::DragValue::new(&mut s.dpi).range(0.0..=2400.0))
                .on_hover_text("used only when width and height are 0; 0 = 96");
            ui.end_row();
            ui.label("Threshold");
            ui.add_enabled(!s.dither, egui::Slider::new(&mut s.threshold, 0..=255));
            ui.end_row();
            ui.label("");
            ui.horizontal(|ui| {
                ui.checkbox(&mut s.dither, "dither");
                ui.checkbox(&mut s.invert, "invert");
            });
            ui.end_row();
        }
        Source::Svg(s) => {
            path_row(ui, &mut s.path, &mut pick, ("SVG", &["svg", "svgz"]));
            ui.label("Width");
            ui.add(mm_value(&mut s.width_mm, 0.0..=200.0)).on_hover_text("0 = document size");
            ui.end_row();
            ui.label("Threshold");
            ui.add(egui::Slider::new(&mut s.threshold, 0..=255));
            ui.end_row();
            ui.label("");
            ui.checkbox(&mut s.invert, "invert (raster)");
            ui.end_row();
            ui.label("Tolerance");
            ui.add(egui::DragValue::new(&mut s.tolerance_mm).speed(0.001).range(0.001..=0.5).suffix(" mm"))
                .on_hover_text("vector mode: curve flattening");
            ui.end_row();
        }
        Source::Qr(q) => {
            ui.label("Data");
            ui.add(egui::TextEdit::multiline(&mut q.data).desired_rows(2).desired_width(200.0));
            ui.end_row();
            ui.label("Module");
            ui.add(mm_value(&mut q.module_mm, 0.05..=5.0));
            ui.end_row();
            ui.label("ECC");
            ui.horizontal(|ui| {
                for l in ["L", "M", "Q", "H"] {
                    if ui.selectable_label(q.ecc.eq_ignore_ascii_case(l), l).clicked() {
                        q.ecc = l.into();
                    }
                }
            });
            ui.end_row();
        }
        Source::Dxf(d) => {
            path_row(ui, &mut d.path, &mut pick, ("DXF", &["dxf"]));
            ui.label("Scale");
            ui.add(egui::DragValue::new(&mut d.scale).speed(0.01).range(0.001..=1000.0));
            ui.end_row();
            ui.label("Layers");
            let mut layers = d.layers.join(", ");
            if ui.add(egui::TextEdit::singleline(&mut layers).hint_text("all")).changed() {
                d.layers = layers.split(',').map(|s| s.trim().to_string()).filter(|s| !s.is_empty()).collect();
            }
            ui.end_row();
            ui.label("Tolerance");
            ui.add(egui::DragValue::new(&mut d.tolerance_mm).speed(0.001).range(0.001..=0.5).suffix(" mm"));
            ui.end_row();
        }
        Source::Shape(s) => {
            ui.label("Shape");
            ui.horizontal(|ui| {
                ui.selectable_value(&mut s.shape, ShapeKind::Rect, "rect");
                ui.selectable_value(&mut s.shape, ShapeKind::Circle, "circle");
                ui.selectable_value(&mut s.shape, ShapeKind::Cross, "cross");
            });
            ui.end_row();
            ui.label(if s.shape == ShapeKind::Circle { "Diameter" } else { "Width" });
            ui.add(mm_value(&mut s.width_mm, 0.05..=200.0));
            ui.end_row();
            if s.shape != ShapeKind::Circle {
                ui.label("Height");
                ui.add(mm_value(&mut s.height_mm, 0.0..=200.0));
                ui.end_row();
            }
        }
        Source::Ruler(r) => {
            ui.label("Length");
            ui.add(mm_value(&mut r.length_mm, 1.0..=200.0));
            ui.end_row();
            ui.label("Step");
            ui.add(mm_value(&mut r.step_mm, 0.1..=10.0));
            ui.end_row();
            ui.label("");
            ui.checkbox(&mut r.labels, "labels every 10 mm");
            ui.end_row();
            ui.label("Font");
            font_picker(ui, &mut r.font);
            ui.end_row();
            ui.label("Label size");
            ui.add(mm_value(&mut r.label_mm, 0.5..=10.0));
            ui.end_row();
        }
    });
    let _ = Mode::Raster;
    pick
}

fn path_row(
    ui: &mut egui::Ui,
    path: &mut String,
    pick: &mut Option<(&'static str, &'static [&'static str])>,
    filter: (&'static str, &'static [&'static str]),
) {
    ui.label("File");
    ui.horizontal(|ui| {
        ui.add(egui::TextEdit::singleline(path).desired_width(150.0));
        if ui.button("…").clicked() {
            *pick = Some(filter);
        }
    });
    ui.end_row();
}

fn font_picker(ui: &mut egui::Ui, font: &mut String) {
    ui.horizontal(|ui| {
        ui.add(egui::TextEdit::singleline(font).desired_width(150.0));
        ui.menu_button("Fonts", |ui| {
            egui::ScrollArea::vertical().max_height(320.0).show(ui, |ui| {
                for f in crate::sources::font_families() {
                    if ui.button(f).clicked() {
                        *font = f.clone();
                        ui.close();
                    }
                }
            });
        });
    });
}
