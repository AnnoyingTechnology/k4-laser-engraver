//! Compile jobs, stream them to the simulator, and replay the exact bytes.

use std::sync::atomic::Ordering;
use std::time::{Duration, Instant};

use k4laser::job::{GeometryCache, Job, Operation, calibration_grid, compile_job};
use k4laser::machine::{Aborted, Control, Machine};
use k4laser::plan::{Geometry, Plan};
use k4laser::preview::render_frames;
use k4laser::protocol::{self as P, Fields};
use k4laser::sources::{ImageProcess, ImageSrc, Mode, QrSrc, ShapeKind, ShapeSrc, Source, SvgSrc, TextSrc};
use k4laser::timing::{Estimate, Timing};
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
fn vector_moves_between_paths_do_not_burn() {
    let text = Source::Text(TextSrc { text: "K4 08".into(), size_mm: 6.0, ..Default::default() });
    let plan = compile(&job(vec![Operation::new(text).mode(Mode::Vector).at(10.0, 10.0)]));
    let op = &plan.ops[0];
    let Geometry::Vector(segs) = &*op.geometry else { panic!("vector expected") };
    assert!(segs.windows(2).filter(|w| w[0][2..] != w[1][..2]).count() >= 4, "needs several paths");
    let sim = SimulatedPort::default();
    run_sim(&plan, &sim).unwrap();
    let (burn, _) = render_frames(&sim.frames(), P::K4_AREA_PX);
    // reference: each segment burnt on its own
    let mut bytes = P::start(op.x as i64, op.y as i64).unwrap();
    for s in segs {
        bytes.extend(P::vector_batch(&P::vector_stream(&[*s], 1), op.power, op.depth).unwrap());
    }
    let (expected, _) = render_frames(&P::split_frames(&bytes).unwrap(), P::K4_AREA_PX);
    assert_eq!(burn.count(), expected.count());
    assert!(burn == expected);
}

#[test]
fn learned_timing_predicts_the_next_run() {
    let text = Source::Text(TextSrc { text: "K4".into(), size_mm: 4.0, ..Default::default() });
    let plan = compile(&job(vec![
        Operation::new(text).at(5.0, 5.0),
        Operation::new(rect(10.0, 5.0)).mode(Mode::Vector).at(30.0, 5.0),
    ]));
    let mut sim = SimulatedPort::default();
    sim.row_time = Duration::from_millis(3);
    let mut m = Machine::new(Box::new(sim));
    m.log = Box::new(|_| {});
    m.connect(None, true).unwrap();
    assert_eq!(Timing::default().estimate(&plan), Estimate::Unknown("raster"));
    m.timing = Some(Timing::default());
    let t0 = Instant::now();
    m.run(&plan, &Control::default(), |_| {}).unwrap();
    let took = t0.elapsed().as_secs_f64();
    let Estimate::Known(est) = m.timing.as_ref().unwrap().estimate(&plan) else { panic!("no estimate") };
    assert!((est.as_secs_f64() - took).abs() < 0.25 * took, "estimated {est:?}, took {took:.3} s");
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
fn passes_repeat_inside_one_job() {
    let ops = |passes| {
        let mut v = Operation::new(rect(10.0, 5.0)).mode(Mode::Vector).at(5.0, 5.0);
        let mut r = Operation::new(rect(1.0, 1.0)).mode(Mode::Raster).at(30.0, 30.0);
        (v.passes, r.passes) = (passes, passes);
        compile(&job(vec![v, r]))
    };
    let (once, twice) = (ops(1), ops(2));
    let sim = SimulatedPort::default();
    run_sim(&twice, &sim).unwrap();
    let frames = sim.frames();
    let names: Vec<String> = frames.iter().map(|f| f.name()).collect();
    // homed once on connect; no homing between jobs, one START per item
    assert_eq!(names.iter().filter(|n| *n == "HOME").count(), 1);
    assert_eq!(names.iter().filter(|n| *n == "START").count(), 2);
    let job2 = names.iter().position(|n| n == "STOP").unwrap() + 1;
    assert_eq!(&names[job2..job2 + 3], ["MOVE_REL", "RESET", "START"]);
    assert_eq!(frames[job2].fields(), Fields::MoveRel { dx: 500, dy: 500 });
    // raster rows burn twice in place
    let rows: Vec<u16> = frames
        .iter()
        .filter_map(|f| match f.fields() {
            Fields::Raster { row, .. } => Some(row),
            _ => None,
        })
        .collect();
    assert_eq!(rows, (0..20).flat_map(|r| [r, r]).collect::<Vec<u16>>());
    // the vector job ends back at its origin, laser off
    let last = frames[..job2]
        .iter()
        .rev()
        .find_map(|f| match f.fields() {
            Fields::Vector { segments, .. } => Some(segments),
            _ => None,
        })
        .unwrap();
    assert_eq!(last[last.len() - 2], P::VECTOR_LASER_OFF);
    assert_eq!(last[last.len() - 1][2..], [0, 0]);
    // and the second pass burns exactly the same pixels
    let sim1 = SimulatedPort::default();
    run_sim(&once, &sim1).unwrap();
    assert!(render_frames(&frames, P::K4_AREA_PX).0 == render_frames(&sim1.frames(), P::K4_AREA_PX).0);
    assert_eq!(twice.data_frames(), frames.iter().filter(|f| f.op == P::OP_RASTER_ROW || f.op == P::OP_VECTOR).count());
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

fn temp_dir(name: &str) -> std::path::PathBuf {
    let dir = std::env::temp_dir().join(format!("k4laser-{name}-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

#[test]
fn grayscale_image_burns_one_pass_per_tone() {
    // five bands, white to black: white never burns, the others are tones 1..4
    let dir = temp_dir("gray");
    let bands = [255u8, 191, 128, 64, 0];
    image::GrayImage::from_fn(50, 10, |x, _| image::Luma([bands[x as usize / 10]])).save(dir.join("g.png")).unwrap();
    let src = Source::Image(ImageSrc {
        path: "g.png".into(),
        width_mm: 5.0,
        process: ImageProcess::Grayscale,
        levels: 4,
        ..Default::default()
    });
    let mut op = Operation::new(src).at(2.0, 2.0);
    op.power = 80.0;
    op.min_power = 20.0;
    let plan = compile(&Job { ops: vec![op], base_dir: dir.clone(), ..Default::default() });
    std::fs::remove_dir_all(&dir).unwrap();
    let powers: Vec<u16> = plan.ops.iter().map(|p| p.power).collect();
    assert_eq!(powers, [350, 500, 650, 800]);
    assert!(plan.ops[3].name.ends_with("tone 4/4"), "{}", plan.ops[3].name);
    // tones are disjoint and lighter tones sit left of darker ones
    let left = |i: usize| plan.ops[i].x;
    assert!(left(0) < left(1) && left(1) < left(2) && left(2) < left(3));
}

#[test]
fn rotation_swaps_dimensions() {
    let mut op = Operation::new(rect(10.0, 4.0)).mode(Mode::Raster).at(5.0, 5.0);
    let flat = compile(&job(vec![op.clone()]));
    op.rotate = 90;
    let turned = compile(&job(vec![op]));
    let (a, b) = (&flat.ops[0], &turned.ops[0]);
    assert_eq!((a.width, a.height), (200, 80));
    assert_eq!((b.width, b.height), (80, 200));
    assert_eq!((b.x, b.y), (100, 100));
}

#[test]
fn svg_user_units_are_css_pixels() {
    let dir = temp_dir("svg");
    let svg =
        r#"<svg xmlns="http://www.w3.org/2000/svg" width="100" height="50"><rect width="100" height="50"/></svg>"#;
    std::fs::write(dir.join("r.svg"), svg).unwrap();
    let src = Source::Svg(SvgSrc { path: "r.svg".into(), ..Default::default() });
    let plan = compile(&Job { ops: vec![Operation::new(src)], base_dir: dir.clone(), ..Default::default() });
    std::fs::remove_dir_all(&dir).unwrap();
    // 100 px = 26.46 mm = 529 machine px
    let op = &plan.ops[0];
    assert!((528..=530).contains(&op.width) && (264..=266).contains(&op.height), "{}x{}", op.width, op.height);
}

#[test]
fn line_mode_perimeters_grow_outward() {
    let mut op = Operation::new(rect(10.0, 5.0)).mode(Mode::Vector).at(5.0, 5.0);
    op.outlines = 3;
    op.outline_spacing_mm = 0.5;
    let plan = compile(&job(vec![op]));
    let p = &plan.ops[0];
    let near = |a: u32, b: u32| a.abs_diff(b) <= 1;
    // two extra rings, 1 mm out on every side
    assert!(near(p.x, 80) && near(p.y, 80) && near(p.width, 240) && near(p.height, 140), "{:?}", p.bbox());
    let Geometry::Vector(segs) = &*p.geometry else { panic!("vector expected") };
    let (burn, _) = p.lengths_mm();
    // 30 + 34 + 38 mm of perimeter, corners rounded slightly shorter
    assert!(segs.len() > 12 && (96.0..=102.5).contains(&burn), "{burn}");
}
