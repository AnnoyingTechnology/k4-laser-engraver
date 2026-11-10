//! Job model in millimetres, TOML persistence, and compilation to a machine plan.

use std::collections::{BTreeMap, HashMap};
use std::path::{Path, PathBuf};
use std::sync::Arc;

use anyhow::{Context, Result, anyhow, bail};
use serde::{Deserialize, Serialize};

use crate::plan::{Geometry, OpPlan, Plan, segments_from_polylines};
use crate::protocol::{self as P};
use crate::sources::{self, Artwork, Mode, ShapeKind, ShapeSrc, Source, TextSrc, mm2px};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Operation {
    #[serde(default)]
    pub name: String,
    #[serde(default)]
    pub mode: Mode,
    /// Top-left of the burnt area, from the work-area origin.
    #[serde(default)]
    pub x_mm: f64,
    #[serde(default)]
    pub y_mm: f64,
    /// Percent, 0.1 resolution on the wire.
    #[serde(default = "default_power")]
    pub power: f64,
    /// Vendor "carving depth" 0..100 (dwell; unit unknown).
    #[serde(default = "default_depth")]
    pub depth: u16,
    #[serde(default = "default_passes")]
    pub passes: u32,
    #[serde(default = "default_true")]
    pub enabled: bool,
    pub source: Source,
}

fn default_power() -> f64 {
    100.0
}
fn default_depth() -> u16 {
    10
}
fn default_passes() -> u32 {
    1
}
fn default_true() -> bool {
    true
}

impl Operation {
    pub fn new(source: Source) -> Self {
        let mode = source.modes()[0];
        Self {
            name: String::new(),
            mode,
            x_mm: 0.0,
            y_mm: 0.0,
            power: default_power(),
            depth: default_depth(),
            passes: 1,
            enabled: true,
            source,
        }
    }

    pub fn at(mut self, x_mm: f64, y_mm: f64) -> Self {
        self.x_mm = x_mm;
        self.y_mm = y_mm;
        self
    }

    pub fn mode(mut self, mode: Mode) -> Self {
        self.mode = mode;
        self
    }

    pub fn label(&self) -> String {
        if !self.name.is_empty() {
            return self.name.clone();
        }
        match &self.source {
            Source::Text(t) => format!("text “{}”", t.text),
            s => match s.path().filter(|p| !p.is_empty()) {
                Some(p) => format!(
                    "{} {}",
                    s.kind(),
                    Path::new(p).file_name().map(|f| f.to_string_lossy()).unwrap_or_default()
                ),
                None => s.kind().to_string(),
            },
        }
    }
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Job {
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub variables: BTreeMap<String, String>,
    #[serde(default, rename = "op")]
    pub ops: Vec<Operation>,
    /// Directory relative source paths are resolved against.
    #[serde(skip)]
    pub base_dir: PathBuf,
}

impl Job {
    pub fn load(path: &Path, overrides: &BTreeMap<String, String>) -> Result<Job> {
        let text = std::fs::read_to_string(path).with_context(|| format!("{}", path.display()))?;
        let mut job: Job = toml::from_str(&text).map_err(|e| anyhow!("{}: {e}", path.display()))?;
        job.variables.extend(overrides.clone());
        job.base_dir = std::path::absolute(path)?.parent().map(Path::to_path_buf).unwrap_or_default();
        Ok(job)
    }

    pub fn to_toml(&self) -> Result<String> {
        let body = toml::to_string_pretty(self)?;
        Ok(format!("# k4laser job — coordinates in mm from the work-area origin (top-left)\n\n{body}"))
    }

    /// Atomic save (write + rename).
    pub fn save(&self, path: &Path) -> Result<()> {
        let tmp = path.with_extension("toml.tmp");
        std::fs::write(&tmp, self.to_toml()?)?;
        std::fs::rename(&tmp, path)?;
        Ok(())
    }
}

// ----------------------------------------------------------- compilation

/// Rendered artwork quantised to machine pixels, relative to the operation origin.
#[derive(Debug)]
pub struct Prepared {
    pub notes: Vec<String>,
    /// `None` when the artwork is blank.
    pub shape: Option<Shape>,
}

#[derive(Debug, Clone)]
pub struct Shape {
    /// Offset of the burnt area from (x_mm, y_mm), px.
    pub dx: i64,
    pub dy: i64,
    pub w: u32,
    pub h: u32,
    pub geometry: Arc<Geometry>,
    pub hash: u64,
}

/// Prepared geometry keyed on the substituted source, mode and file mtime:
/// moving an operation or changing its power never re-renders it.
#[derive(Default)]
pub struct GeometryCache {
    map: HashMap<String, Arc<Prepared>>,
}

impl GeometryCache {
    pub fn get(&mut self, op: &Operation, base_dir: &Path, vars: &BTreeMap<String, String>) -> Result<Arc<Prepared>> {
        let src = op.source.substituted(vars);
        let key = format!(
            "{}|{}|{:?}|{}",
            toml::to_string(&src).unwrap_or_default(),
            op.mode.as_str(),
            sources::source_mtime(&src, base_dir),
            base_dir.display()
        );
        if let Some(r) = self.map.get(&key) {
            return Ok(r.clone());
        }
        let rendered = sources::render(&src, op.mode, base_dir)?;
        let prepared = Arc::new(Prepared { shape: quantise(&rendered.art), notes: rendered.notes });
        if self.map.len() > 64 {
            self.map.clear();
        }
        self.map.insert(key, prepared.clone());
        Ok(prepared)
    }
}

/// Raster: crop to ink. Vector: mm -> px, drop repeats, order, split into segments.
pub fn quantise(art: &Artwork) -> Option<Shape> {
    let (dx, dy, w, h, geometry) = match art {
        Artwork::Raster(bm) => {
            let ((cx, cy), crop) = bm.crop_to_ink()?;
            (cx as i64, cy as i64, crop.width as u32, crop.height as u32, Geometry::Raster(crop))
        }
        Artwork::Vector(polys) => {
            let mut px: Vec<Vec<(i64, i64)>> = Vec::new();
            for pl in polys {
                let mut pts: Vec<(i64, i64)> = Vec::new();
                for &(x, y) in pl {
                    let p = (mm2px(x), mm2px(y));
                    if pts.last() != Some(&p) {
                        pts.push(p);
                    }
                }
                if pts.len() >= 2 {
                    px.push(pts);
                }
            }
            let x0 = px.iter().flatten().map(|p| p.0).min()?;
            let y0 = px.iter().flatten().map(|p| p.1).min()?;
            let x1 = px.iter().flatten().map(|p| p.0).max()?;
            let y1 = px.iter().flatten().map(|p| p.1).max()?;
            if x1 - x0 > u16::MAX as i64 || y1 - y0 > u16::MAX as i64 {
                return None;
            }
            let rel: Vec<Vec<(u16, u16)>> =
                px.iter().map(|pl| pl.iter().map(|&(x, y)| ((x - x0) as u16, (y - y0) as u16)).collect()).collect();
            let segs = segments_from_polylines(&rel);
            let w = segs.iter().map(|s| s[0].max(s[2])).max().unwrap_or(0) as u32;
            let h = segs.iter().map(|s| s[1].max(s[3])).max().unwrap_or(0) as u32;
            (x0, y0, w, h, Geometry::Vector(segs))
        }
    };
    let hash = geometry.hash64();
    Some(Shape { dx, dy, w, h, geometry: Arc::new(geometry), hash })
}

/// Validate process parameters and place prepared geometry at the operation position.
pub fn compile_op(op: &Operation, prepared: &Prepared) -> Result<Option<OpPlan>> {
    let label = op.label();
    if !(0.0..=100.0).contains(&op.power) {
        bail!("{label}: power {} % outside 0..100", op.power);
    }
    if op.depth > P::DEPTH_MAX {
        bail!("{label}: depth {} outside 0..{}", op.depth, P::DEPTH_MAX);
    }
    if !(1..=50).contains(&op.passes) {
        bail!("{label}: passes {} outside 1..50", op.passes);
    }
    let Some(s) = &prepared.shape else { return Ok(None) };
    let (x, y) = (mm2px(op.x_mm) + s.dx, mm2px(op.y_mm) + s.dy);
    if x < 0 || y < 0 || x + s.w as i64 > u16::MAX as i64 || y + s.h as i64 > u16::MAX as i64 {
        bail!(
            "{label}: position ({:.2}, {:.2}) mm is outside the machine range",
            x as f64 * P::PITCH_MM,
            y as f64 * P::PITCH_MM
        );
    }
    Ok(Some(OpPlan {
        name: label,
        x: x as u32,
        y: y as u32,
        width: s.w,
        height: s.h,
        power: (op.power * 10.0).round() as u16,
        depth: op.depth,
        passes: op.passes,
        geometry: s.geometry.clone(),
        geom_hash: s.hash,
    }))
}

/// Error message when `p` leaves the work area.
pub fn check_area(p: &OpPlan, area_px: (u32, u32)) -> Result<()> {
    if p.x + p.width > area_px.0 || p.y + p.height > area_px.1 {
        bail!(
            "{}: spans ({:.2}, {:.2})–({:.2}, {:.2}) mm, outside the {}x{} mm work area",
            p.name,
            p.x as f64 * P::PITCH_MM,
            p.y as f64 * P::PITCH_MM,
            (p.x + p.width) as f64 * P::PITCH_MM,
            (p.y + p.height) as f64 * P::PITCH_MM,
            area_px.0 as f64 * P::PITCH_MM,
            area_px.1 as f64 * P::PITCH_MM
        );
    }
    Ok(())
}

/// Render and quantise enabled operations; fails when anything leaves the work area.
pub fn compile_job(
    job: &Job,
    area_px: (u32, u32),
    only: Option<&[usize]>,
    cache: &mut GeometryCache,
) -> Result<(Plan, Vec<String>)> {
    let mut plan = Plan::default();
    let mut notes = Vec::new();
    for (i, op) in job.ops.iter().enumerate() {
        if !op.enabled || only.is_some_and(|o| !o.contains(&i)) {
            continue;
        }
        let prepared = cache.get(op, &job.base_dir, &job.variables).with_context(|| op.label())?;
        notes.extend(prepared.notes.iter().map(|n| format!("{}: {n}", op.label())));
        let Some(p) = compile_op(op, &prepared)? else {
            notes.push(format!("{}: nothing to burn, skipped", op.label()));
            continue;
        };
        check_area(&p, area_px)?;
        plan.ops.push(p);
    }
    Ok((plan, notes))
}

// ------------------------------------------------------------ generators

/// Power x depth calibration matrix: filled squares with row/column labels.
pub fn calibration_grid(powers: &[f64], depths: &[u16], cell_mm: f64, gap_mm: f64) -> Vec<Operation> {
    let (x_mm, y_mm, label_mm) = (1.0, 1.0, 1.6);
    let font = "DejaVu Sans Mono";
    let r = |v: f64| (v * 1e6).round() / 1e6; // keep the saved TOML free of float noise
    let label = |text: String, x: f64, y: f64, name: String| {
        let mut op =
            Operation::new(Source::Text(TextSrc { text, font: font.into(), size_mm: label_mm, ..Default::default() }))
                .at(r(x), r(y));
        op.power = 60.0;
        op.depth = 5;
        op.name = name;
        op
    };
    let x0 = x_mm + label_mm * 2.2 + gap_mm;
    let y0 = y_mm + label_mm + gap_mm;
    let mut ops = Vec::new();
    for (j, p) in powers.iter().enumerate() {
        ops.push(label(format!("{p}"), x0 + j as f64 * (cell_mm + gap_mm), y_mm, format!("label P{p}")));
    }
    for (i, &d) in depths.iter().enumerate() {
        let y = y0 + i as f64 * (cell_mm + gap_mm);
        ops.push(label(format!("{d}"), x_mm, y + cell_mm / 2.0 - label_mm / 2.0, format!("label D{d}")));
        for (j, &p) in powers.iter().enumerate() {
            let shape = ShapeSrc { shape: ShapeKind::Rect, width_mm: cell_mm, height_mm: cell_mm };
            let mut op =
                Operation::new(Source::Shape(shape)).mode(Mode::Raster).at(r(x0 + j as f64 * (cell_mm + gap_mm)), r(y));
            op.power = p;
            op.depth = d;
            op.name = format!("cell P{p} D{d}");
            ops.push(op);
        }
    }
    ops
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn toml_roundtrip_and_overrides() {
        let mut op = Operation::new(Source::Text(TextSrc {
            text: "Tag \"${serial}\"".into(),
            size_mm: 2.5,
            ..Default::default()
        }))
        .at(1.25, 2.0);
        op.power = 55.5;
        op.depth = 12;
        op.passes = 2;
        op.name = "tag".into();
        let job = Job {
            variables: [("serial".to_string(), "0042".to_string())].into(),
            ops: vec![op],
            base_dir: PathBuf::new(),
        };
        let dir = std::env::temp_dir().join(format!("k4laser-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("job.toml");
        job.save(&path).unwrap();
        let back = Job::load(&path, &[("serial".to_string(), "0099".to_string())].into()).unwrap();
        std::fs::remove_dir_all(&dir).unwrap();
        assert_eq!(back.ops, job.ops);
        assert_eq!(back.variables["serial"], "0099");
    }

    #[test]
    fn unknown_keys_are_rejected() {
        let bad = "[[op]]\nx_mm = 1\npowr = 50\n[op.source]\ntype = \"shape\"\n";
        assert!(toml::from_str::<Job>(bad).is_err());
        let bad_src = "[[op]]\n[op.source]\ntype = \"shape\"\nwidht_mm = 3\n";
        assert!(toml::from_str::<Job>(bad_src).is_err());
        let ok = "[[op]]\n[op.source]\ntype = \"shape\"\nwidth_mm = 3\n";
        assert!(toml::from_str::<Job>(ok).is_ok());
    }

    #[test]
    fn out_of_area_is_refused() {
        let op = Operation::new(Source::Shape(ShapeSrc::default())).at(75.0, 0.0);
        let job = Job { ops: vec![op], ..Default::default() };
        let err = compile_job(&job, P::K4_AREA_PX, None, &mut GeometryCache::default()).unwrap_err();
        assert!(err.to_string().contains("outside"), "{err}");
    }
}
