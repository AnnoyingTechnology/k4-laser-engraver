//! `k4laser` command line; without a subcommand it opens the GUI.

use std::collections::BTreeMap;
use std::io::Write;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use clap::{Parser, Subcommand};

use k4laser::job::{GeometryCache, Job, calibration_grid, compile_job};
use k4laser::machine::{Aborted, Control, Dir, Machine, Progress, hex};
use k4laser::plan::{Plan, Rect};
use k4laser::protocol as P;
use k4laser::transport::{SerialPort, SimulatedPort, list_ports};
use k4laser::{config, gui, preview, timing};

#[derive(Parser)]
#[command(name = "k4laser", version, about = "Linux driver for K4-class USB mini laser engravers")]
struct Cli {
    /// Log every protocol byte.
    #[arg(short, long, global = true)]
    verbose: bool,
    #[command(subcommand)]
    cmd: Option<Cmd>,
}

#[derive(clap::Args, Clone)]
struct PortArgs {
    /// Serial device, preferably /dev/serial/by-path/… (default: `port` in the config file).
    #[arg(long)]
    port: Option<PathBuf>,
    /// Append every transmitted frame to this file.
    #[arg(long)]
    trace: Option<PathBuf>,
}

#[derive(clap::Args, Clone)]
struct JobArgs {
    /// Job file (.toml).
    job: PathBuf,
    /// Only operation N (1-based); repeatable.
    #[arg(long = "op")]
    ops: Vec<usize>,
    /// Substitute ${KEY} in source strings.
    #[arg(long = "set", value_name = "KEY=VALUE")]
    vars: Vec<String>,
}

#[derive(Subcommand)]
enum Cmd {
    /// List serial ports with stable paths and the processes holding them.
    Ports,
    /// Handshake + model query; no motion, no settings change.
    Probe(PortArgs),
    /// Return the head to the origin.
    Home(PortArgs),
    /// Move the head relatively (mm).
    Jog {
        #[command(flatten)]
        port: PortArgs,
        #[arg(allow_hyphen_values = true)]
        dx: f64,
        #[arg(allow_hyphen_values = true)]
        dy: f64,
    },
    /// Trace the job's bounding box with the low-power beam.
    Frame {
        #[command(flatten)]
        port: PortArgs,
        #[command(flatten)]
        job: JobArgs,
        /// Frame for N seconds instead of until Enter.
        #[arg(long)]
        seconds: Option<f64>,
    },
    /// Frame, confirm, engrave.
    Burn {
        #[command(flatten)]
        port: PortArgs,
        #[command(flatten)]
        job: JobArgs,
        /// Skip the framing step.
        #[arg(long)]
        no_frame: bool,
        /// Do not ask for confirmation.
        #[arg(long)]
        yes: bool,
    },
    /// Stream the job to a simulated machine and render what it would burn.
    Simulate {
        #[command(flatten)]
        job: JobArgs,
        /// Preview PNG (1 px = 0.05 mm, 508 dpi).
        #[arg(short, long)]
        output: Option<PathBuf>,
        /// Write the exact byte stream to this file.
        #[arg(long)]
        trace: Option<PathBuf>,
    },
    /// Decode a byte-stream trace.
    Decode { file: PathBuf },
    /// Write a power x depth calibration job.
    Testgrid {
        #[arg(short, long, default_value = "-")]
        output: String,
        #[arg(long, default_value = "20,40,60,80,100", value_delimiter = ',')]
        powers: Vec<f64>,
        #[arg(long, default_value = "5,10,20,40,80", value_delimiter = ',')]
        depths: Vec<u16>,
        /// Cell size, mm.
        #[arg(long, default_value_t = 4.0)]
        cell: f64,
        /// Gap, mm.
        #[arg(long, default_value_t = 1.5)]
        gap: f64,
    },
    /// Send the hidden 0xFD/0xFE command (experimental, purpose unconfirmed).
    Aux {
        #[command(flatten)]
        port: PortArgs,
        #[arg(value_parser = ["on", "off"])]
        state: String,
    },
    /// Graphical interface (default when no command is given).
    Gui { job: Option<PathBuf> },
}

static INTERRUPTED: AtomicBool = AtomicBool::new(false);

extern "C" fn on_sigint(_: libc::c_int) {
    INTERRUPTED.store(true, Ordering::SeqCst);
}

/// Ctrl-C sets a flag instead of killing the process, so STOP / FRAME_OFF can be sent.
fn trap_sigint() {
    // SAFETY: the handler only stores to an atomic.
    unsafe {
        libc::signal(libc::SIGINT, on_sigint as *const () as libc::sighandler_t);
    }
}

fn interrupted() -> bool {
    INTERRUPTED.load(Ordering::SeqCst)
}

fn mm_to_px(mm: f64) -> i64 {
    (mm / P::PITCH_MM).round() as i64
}

fn parse_vars(pairs: &[String]) -> Result<BTreeMap<String, String>> {
    pairs
        .iter()
        .map(|p| match p.split_once('=') {
            Some((k, v)) => Ok((k.to_string(), v.to_string())),
            None => bail!("--set expects KEY=VALUE, got {p:?}"),
        })
        .collect()
}

fn resolve_port(args: &PortArgs, cfg: &config::Config) -> Result<PathBuf> {
    if let Some(p) = args.port.clone().or_else(|| cfg.port.clone()) {
        return Ok(p);
    }
    let mut msg = format!(
        "no port given: pass --port or set `port` in {}\ncandidates (by-path names are stable; by-id is not for CH340):",
        config::path().display()
    );
    for p in list_ports().iter().filter(|p| p.is_ch340()) {
        msg += &format!("\n  {}   [{}]", p.stable_path().display(), p.label());
    }
    bail!(msg)
}

fn open_machine(args: &PortArgs, cfg: &config::Config, verbose: bool) -> Result<Machine> {
    let path = resolve_port(args, cfg)?;
    let port = SerialPort::open(&path, false).with_context(|| format!("{}", path.display()))?;
    let mut m = Machine::new(Box::new(port));
    let mut trace_file = match &args.trace {
        Some(p) => Some(std::fs::OpenOptions::new().create(true).append(true).open(p)?),
        None => None,
    };
    if verbose || trace_file.is_some() {
        m.trace = Some(Box::new(move |dir, data| {
            if verbose {
                eprintln!("{} {}", if dir == Dir::Tx { "tx" } else { "rx" }, hex(data));
            }
            if let (Dir::Tx, Some(f)) = (dir, trace_file.as_mut()) {
                let _ = f.write_all(data);
            }
        }));
    }
    Ok(m)
}

fn connect(m: &mut Machine, cfg: &config::Config) -> Result<()> {
    let model = m.connect(Some(&cfg.settings), true)?;
    println!("connected: {}", model.describe());
    Ok(())
}

fn load_and_compile(args: &JobArgs, area: (u32, u32)) -> Result<(Job, Plan)> {
    let job = Job::load(&args.job, &parse_vars(&args.vars)?)?;
    let only: Vec<usize> = args.ops.iter().map(|n| n.saturating_sub(1)).collect();
    let (plan, notes) =
        compile_job(&job, area, (!only.is_empty()).then_some(&only[..]), &mut GeometryCache::default())?;
    for n in notes {
        println!("note: {n}");
    }
    for line in plan.summary() {
        println!("  {line}");
    }
    if plan.ops.is_empty() {
        bail!("nothing to burn");
    }
    Ok((job, plan))
}

/// Block until Enter, Ctrl-C or the deadline (stdin is read on a helper thread so Ctrl-C stays responsive).
fn wait_enter_or_interrupt(seconds: Option<f64>) {
    let deadline = seconds.map(|s| Instant::now() + Duration::from_secs_f64(s));
    let (tx, rx) = mpsc::channel();
    if deadline.is_none() {
        std::thread::spawn(move || {
            let mut s = String::new();
            let _ = std::io::stdin().read_line(&mut s);
            let _ = tx.send(());
        });
    }
    while !interrupted() && rx.recv_timeout(Duration::from_millis(50)).is_err() {
        if deadline.is_some_and(|d| Instant::now() >= d) {
            return;
        }
    }
}

fn frame_until_enter(m: &mut Machine, rect: Rect, seconds: Option<f64>) -> Result<()> {
    println!("framing {} — low-power beam is ON", rect.describe_mm());
    m.frame_start(rect)?;
    if seconds.is_none() {
        print!("press Enter to stop framing… ");
        std::io::stdout().flush()?;
    }
    wait_enter_or_interrupt(seconds);
    m.frame_stop()?;
    if interrupted() {
        bail!("interrupted: framing stopped");
    }
    Ok(())
}

fn print_progress(p: Progress) {
    let eta = p.eta.map(|e| format!(", ETA {:5.0} s", e.as_secs_f64())).unwrap_or_default();
    let label: String = p.label.chars().take(60).collect();
    print!(
        "\r{}/{} ({:5.1} %), {:5.0} s{eta}  {label}   ",
        p.step,
        p.total,
        100.0 * p.step as f64 / p.total.max(1) as f64,
        p.elapsed.as_secs_f64()
    );
    let _ = std::io::stdout().flush();
}

fn simulate(plan: &Plan) -> Result<(SimulatedPort, Duration)> {
    let sim = SimulatedPort::default();
    let mut m = Machine::new(Box::new(sim.clone()));
    m.log = Box::new(|_| {});
    m.connect(None, true)?;
    let t0 = Instant::now();
    m.run(plan, &Control::default(), |_| {})?;
    Ok((sim, t0.elapsed()))
}

fn run() -> Result<()> {
    let cli = Cli::parse();
    let verbose = cli.verbose;
    match cli.cmd.unwrap_or(Cmd::Gui { job: None }) {
        Cmd::Ports => {
            for p in list_ports() {
                let mark = if p.is_ch340() { "CH340" } else { "     " };
                println!("{mark}  {}\n        {}", p.stable_path().display(), p.label());
            }
            println!("\nby-id links are unusable for CH340 (no serial number): select ports by-path.");
        }
        Cmd::Probe(port) => {
            let mut m = open_machine(&port, &config::load()?, verbose)?;
            let model = m.probe()?;
            println!("{}: K4 protocol handshake OK, {}", m.port_name(), model.describe());
        }
        Cmd::Home(port) => {
            let cfg = config::load()?;
            let mut m = open_machine(&port, &cfg, verbose)?;
            connect(&mut m, &cfg)?;
            println!("homed");
        }
        Cmd::Jog { port, dx, dy } => {
            let cfg = config::load()?;
            let mut m = open_machine(&port, &cfg, verbose)?;
            connect(&mut m, &cfg)?;
            m.jog(mm_to_px(dx), mm_to_px(dy))?;
            let (x, y) = m.head.unwrap_or_default();
            println!("head at ({:.2}, {:.2}) mm", x as f64 * P::PITCH_MM, y as f64 * P::PITCH_MM);
        }
        Cmd::Frame { port, job, seconds } => {
            trap_sigint();
            let cfg = config::load()?;
            let mut m = open_machine(&port, &cfg, verbose)?;
            connect(&mut m, &cfg)?;
            let (_, plan) = load_and_compile(&job, m.area_px)?;
            frame_until_enter(&mut m, plan.bbox().unwrap(), seconds)?;
        }
        Cmd::Burn { port, job, no_frame, yes } => {
            trap_sigint();
            let cfg = config::load()?;
            let mut m = open_machine(&port, &cfg, verbose)?;
            connect(&mut m, &cfg)?;
            let (_, plan) = load_and_compile(&job, m.area_px)?;
            let learned = timing::load();
            println!("estimated duration: {}", learned.estimate(&plan));
            m.timing = Some(learned);
            if !no_frame {
                frame_until_enter(&mut m, plan.bbox().unwrap(), None)?;
            }
            if !yes {
                print!("Laser safety: goggles on, shield in place, extraction on.\nType BURN to start: ");
                std::io::stdout().flush()?;
                let mut answer = String::new();
                std::io::stdin().read_line(&mut answer)?;
                if answer.trim() != "BURN" || interrupted() {
                    bail!("aborted");
                }
            }
            let control = Control::new();
            let watcher = {
                let control = control.clone();
                std::thread::spawn(move || {
                    while !control.cancelled() {
                        if interrupted() {
                            control.cancel.store(true, Ordering::SeqCst);
                        }
                        std::thread::sleep(Duration::from_millis(50));
                    }
                })
            };
            let t0 = Instant::now();
            let res = m.run(&plan, &control, print_progress);
            control.cancel.store(true, Ordering::SeqCst);
            let _ = watcher.join();
            println!();
            if let Some(t) = &m.timing
                && let Err(e) = timing::save(t)
            {
                eprintln!("warning: timing not saved: {e:#}");
            }
            match res {
                Ok(()) => println!("done in {:.0} s", t0.elapsed().as_secs_f64()),
                Err(e) if e.is::<Aborted>() => bail!("interrupted: STOP sent"),
                Err(e) => return Err(e),
            }
        }
        Cmd::Simulate { job, output, trace } => {
            let (_, plan) = load_and_compile(&job, P::K4_AREA_PX)?;
            println!("estimated duration on the machine: {}", timing::load().estimate(&plan));
            let (sim, elapsed) = simulate(&plan)?;
            let sent = sim.sent();
            let frames = P::split_frames(&sent)?;
            let (burn, stats) = preview::render_frames(&frames, P::K4_AREA_PX);
            println!(
                "{} frames, {} bytes, streamed in {:.2} s; {stats:?}",
                frames.len(),
                sent.len(),
                elapsed.as_secs_f64()
            );
            if let Some(out) = output {
                preview::save_png(&burn, &out, plan.bbox())?;
                println!("preview: {} (1 px = 0.05 mm)", out.display());
            }
            if let Some(t) = trace {
                std::fs::write(&t, &sent)?;
                println!("trace: {} ({} bytes, decode with `k4laser decode`)", t.display(), sent.len());
            }
        }
        Cmd::Decode { file } => {
            let frames = P::split_frames(&std::fs::read(&file)?)?;
            for (i, fr) in frames.iter().enumerate() {
                let head = hex(&fr.data[..fr.data.len().min(12)]);
                println!("{i:6}  {head:36}  {}", fr.describe());
            }
        }
        Cmd::Testgrid { output, powers, depths, cell, gap } => {
            let job = Job { ops: calibration_grid(&powers, &depths, cell, gap), ..Default::default() };
            let text = job.to_toml()?;
            if output == "-" {
                print!("{text}");
            } else {
                std::fs::write(&output, text)?;
                println!("wrote {output}: {} powers x {} depths", powers.len(), depths.len());
            }
        }
        Cmd::Aux { port, state } => {
            let mut m = open_machine(&port, &config::load()?, verbose)?;
            m.probe()?;
            let on = state == "on";
            println!("sending hidden V2.7 command {} (purpose unconfirmed)", if on { "0xFD" } else { "0xFE" });
            println!("{}", if m.aux(on)? { "ACK received" } else { "no ACK" });
        }
        Cmd::Gui { job } => gui::run(job)?,
    }
    Ok(())
}

fn main() {
    if let Err(e) = run() {
        eprintln!("error: {e:#}");
        std::process::exit(if interrupted() { 130 } else { 1 });
    }
}
