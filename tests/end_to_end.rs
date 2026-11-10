//! Compile jobs, stream them to the simulator, and replay the exact bytes.

use std::sync::atomic::Ordering;
use std::time::Duration;

use k4laser::job::{GeometryCache, Job, Operation, calibration_grid, compile_job};
use k4laser::machine::{Aborted, Control, Machine};
use k4laser::plan::{Geometry, Plan};
use k4laser::preview::render_frames;
use k4laser::protocol::{self as P, Fields};
use k4laser::sources::{Mode, QrSrc, ShapeKind, ShapeSrc, Source, TextSrc};
use k4laser::transport::SimulatedPort;

fn compile(job: &Job) -> Plan {
    compile_job(job, P::K4_AREA_PX, None, &mut GeometryCache::default()).unwrap().0
}

fn run_sim(plan: &Plan, sim: &SimulatedPort) -> anyhow::Result<()> {
    let mut m = Machine::new(Box::new(sim.clone()));
    m.log = Box::new(|_| {});
    m.connect(None, true)?;
    m.run(plan, &Control::default(), |_| {})
}

fn rect(w: f64, h: f64) -> Source {
    Source::Shape(ShapeSrc { shape: ShapeKind::Rect, width_mm: w, height_mm: h })
}

fn job(ops: Vec<Operation>) -> Job {
    Job { ops, ..Default::default() }
}

#[test]
fn raster_text_matches_stream() {
    let text = Source::Text(TextSrc { text: "K4 0.05".into(), size_mm: 3.0, ..Default::default() });
    let plan = compile(&job(vec![Operation::new(text).at(10.0, 7.5)]));
    let sim = SimulatedPort::default();
    run_sim(&plan, &sim).unwrap();
    let op = &plan.ops[0];
    let (burn, stats) = render_frames(&sim.frames(), P::K4_AREA_PX);
    let Geometry::Raster(bm) = &*op.geometry else { panic!("raster expected") };
    // every burnt pixel in the replay is exactly the op bitmap at its position
    assert_eq!(burn.count(), bm.count());
    for y in 0..bm.height {
        for x in 0..bm.width {
            assert_eq!(burn.get(op.x as usize + x, op.y as usize + y), bm.get(x, y));
        }
    }
    assert_eq!(stats.out_of_area, 0);
    // the ink box starts at the requested placement
    assert_eq!((op.x, op.y), (200, 150));
    // 7 monospace advances of 0.6 em at 3 mm ≈ 12.3 mm; cap height ≈ 0.73 em
    assert!((230..260).contains(&op.width) && (40..50).contains(&op.height), "{}x{}", op.width, op.height);
    assert_eq!(sim.frames().last().unwrap().op, P::OP_STOP);
}

#[test]
fn vector_rect_is_exact() {
    let plan = compile(&job(vec![Operation::new(rect(10.0, 5.0)).mode(Mode::Vector).at(5.0, 5.0)]));
    let op = &plan.ops[0];
    assert_eq!((op.x, op.y, op.width, op.height), (100, 100, 200, 100));
    let sim = SimulatedPort::default();
    run_sim(&plan, &sim).unwrap();
    let (burn, _) = render_frames(&sim.frames(), P::K4_AREA_PX);
    assert!((100..=300).all(|x| burn.get(x, 100) && burn.get(x, 200)));
    assert!((100..=200).all(|y| burn.get(100, y) && burn.get(300, y)));
    assert_eq!(burn.count(), 2 * 201 + 2 * 99);
}

#[test]
fn job_sequence_matches_vendor_order() {
    let plan = compile(&job(vec![Operation::new(rect(1.0, 1.0)).mode(Mode::Raster).at(2.0, 3.0)]));
    let sim = SimulatedPort::default();
    run_sim(&plan, &sim).unwrap();
    let names: Vec<String> = sim.frames().iter().map(|f| f.name()).collect();
    assert_eq!(&names[..6], ["HANDSHAKE", "MODEL", "HOME", "MOVE_REL", "RESET", "START"]);
    assert_eq!(sim.frames()[3].fields(), Fields::MoveRel { dx: 40, dy: 60 });
    assert_eq!(sim.frames()[5].fields(), Fields::Start { x: 40, y: 60 });
    assert_eq!(names.iter().filter(|n| *n == "RASTER_ROW").count(), 20);
    assert_eq!(names.last().unwrap(), "STOP");
}

#[test]
fn nak_triggers_resend() {
    let plan = compile(&job(vec![Operation::new(rect(2.0, 2.0)).mode(Mode::Raster).at(1.0, 1.0)]));
    let mut sim = SimulatedPort::default();
    sim.nak_every = 3;
    run_sim(&plan, &sim).unwrap();
    let rows: Vec<u16> = sim
        .frames()
        .iter()
        .filter_map(|f| match f.fields() {
            Fields::Raster { row, .. } => Some(row),
            _ => None,
        })
        .collect();
    assert!(rows.len() > plan.ops[0].frame_count());
    let mut uniq = rows.clone();
    uniq.dedup();
    assert_eq!(uniq, (0..40).collect::<Vec<u16>>());
}

#[test]
fn cancel_sends_stop() {
    let plan = compile(&job(vec![Operation::new(rect(20.0, 20.0)).mode(Mode::Raster).at(1.0, 1.0)]));
    let mut sim = SimulatedPort::default();
    sim.row_time = Duration::from_millis(5);
    let mut m = Machine::new(Box::new(sim.clone()));
    m.log = Box::new(|_| {});
    m.connect(None, true).unwrap();
    let control = Control::new();
    let c2 = control.clone();
    std::thread::spawn(move || {
        std::thread::sleep(Duration::from_millis(100));
        c2.cancel.store(true, Ordering::SeqCst);
    });
    let err = m.run(&plan, &control, |_| {}).unwrap_err();
    assert!(err.is::<Aborted>());
    let frames = sim.frames();
    assert_eq!(frames.last().unwrap().op, P::OP_STOP);
    let rows = frames.iter().filter(|f| f.op == P::OP_RASTER_ROW).count();
    assert!(rows > 0 && rows < 400, "{rows}");
}

#[test]
fn framing_moves_then_traces_and_stops() {
    let sim = SimulatedPort::default();
    let mut m = Machine::new(Box::new(sim.clone()));
    m.log = Box::new(|_| {});
    m.connect(None, true).unwrap();
    let r = k4laser::plan::Rect { x: 100, y: 200, w: 300, h: 50 };
    m.frame_start(r).unwrap();
    m.frame_start(k4laser::plan::Rect { x: 110, ..r }).unwrap();
    drop(m); // dropping a framing session must switch the beam off
    let tail: Vec<String> = sim.frames().iter().skip(3).map(|f| f.describe()).collect();
    assert_eq!(
        tail,
        [
            "MOVE_REL dx=100 dy=200",
            "FRAME_ON width=300 height=50 x=0 y=0",
            "FRAME_OFF",
            "MOVE_REL dx=10 dy=0",
            "FRAME_ON width=300 height=50 x=0 y=0",
            "STOP",
            "FRAME_OFF",
        ]
    );
}

#[test]
fn out_of_area_point_refused_by_machine() {
    let sim = SimulatedPort::default();
    let mut m = Machine::new(Box::new(sim));
    m.log = Box::new(|_| {});
    m.connect(None, true).unwrap();
    assert!(m.frame_start(k4laser::plan::Rect { x: 1500, y: 0, w: 200, h: 10 }).is_err());
}

#[test]
fn calibration_grid_compiles() {
    let plan = compile(&job(calibration_grid(&[20.0, 60.0, 100.0], &[5, 20], 4.0, 1.5)));
    let cells: Vec<_> = plan.ops.iter().filter(|o| o.name.starts_with("cell")).collect();
    assert_eq!(cells.len(), 6);
    let mut pd: Vec<(u16, u16)> = cells.iter().map(|c| (c.power, c.depth)).collect();
    pd.sort();
    assert_eq!(pd, [(200, 5), (200, 20), (600, 5), (600, 20), (1000, 5), (1000, 20)]);
}

#[test]
fn qr_modules_are_whole_pixels() {
    let qr = Source::Qr(QrSrc { data: "HELLO".into(), module_mm: 0.5, ecc: "M".into() });
    let plan = compile(&job(vec![Operation::new(qr).at(1.0, 1.0)]));
    let op = &plan.ops[0];
    // version 1 = 21 modules of 10 px
    assert_eq!((op.width, op.height), (210, 210));
}

#[test]
fn vector_text_outlines_are_closed() {
    let text = Source::Text(TextSrc { text: "O".into(), size_mm: 5.0, ..Default::default() });
    let plan = compile(&job(vec![Operation::new(text).mode(Mode::Vector).at(1.0, 1.0)]));
    let Geometry::Vector(segs) = &*plan.ops[0].geometry else { panic!() };
    assert!(segs.len() > 20);
    // 'O' has two contours; each closes on itself
    let starts: Vec<(u16, u16)> = segs.iter().map(|s| (s[0], s[1])).collect();
    let ends: Vec<(u16, u16)> = segs.iter().map(|s| (s[2], s[3])).collect();
    for e in &ends {
        assert!(starts.contains(e), "open end at {e:?}");
    }
}

#[test]
fn fingerprint_tracks_geometry_not_power() {
    let mut op = Operation::new(rect(3.0, 3.0)).at(1.0, 1.0);
    let a = compile(&job(vec![op.clone()])).fingerprint();
    op.power = 40.0;
    assert_eq!(a, compile(&job(vec![op.clone()])).fingerprint());
    op.x_mm += 0.05;
    assert_ne!(a, compile(&job(vec![op])).fingerprint());
}

#[test]
fn examples_compile() {
    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("examples");
    let mut n = 0;
    for entry in std::fs::read_dir(dir).unwrap() {
        let path = entry.unwrap().path();
        if path.extension().is_some_and(|e| e == "toml") {
            let job = Job::load(&path, &Default::default()).unwrap();
            let plan = compile(&job);
            assert!(!plan.ops.is_empty(), "{}", path.display());
            n += 1;
        }
    }
    assert!(n >= 3);
}
