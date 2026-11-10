//! Artwork sources rendered at the machine pitch (0.05 mm).
//!
//! Rendering yields either a raster (bool bitmap, `true` = burn) or vector
//! polylines in mm (Y down, bounding box at the origin). Text and SVG go
//! through usvg/resvg with system fonts; nothing is shelled out.

use std::path::{Path, PathBuf};
use std::sync::{Arc, OnceLock};

use anyhow::{Context, Result, anyhow, bail};
use resvg::tiny_skia;
use serde::{Deserialize, Serialize};
use usvg::fontdb;

use crate::plan::Bitmap;
use crate::protocol::{PITCH_MM, PX_PER_MM};

pub type Polyline = Vec<(f64, f64)>;

pub fn mm2px(mm: f64) -> i64 {
    (mm * PX_PER_MM).round_ties_even() as i64
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Mode {
    #[default]
    Raster,
    Vector,
}

impl Mode {
    pub fn as_str(self) -> &'static str {
        match self {
            Mode::Raster => "raster",
            Mode::Vector => "vector",
        }
    }

    /// Name shown to people: raster fills areas, vector follows paths.
    pub fn label(self) -> &'static str {
        match self {
            Mode::Raster => "Fill",
            Mode::Vector => "Line",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Align {
    #[default]
    Left,
    Center,
    Right,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct TextSrc {
    pub text: String,
    /// Family, optionally followed by style words: "DejaVu Sans Mono Bold".
    pub font: String,
    /// Em size.
    pub size_mm: f64,
    pub align: Align,
}

impl Default for TextSrc {
    fn default() -> Self {
        Self { text: "TEXT".into(), font: "DejaVu Sans Mono Bold".into(), size_mm: 4.0, align: Align::Left }
    }
}

/// How an image's shades become burns (the machine itself is on/off per pixel).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ImageProcess {
    /// Black and white: pixels darker than `threshold` burn.
    Threshold,
    /// Floyd–Steinberg error diffusion: shades become dot density.
    #[default]
    Dither,
    /// `levels` tones, one pass each, burnt at a power scaled to the tone.
    Grayscale,
}

impl ImageProcess {
    pub fn label(self) -> &'static str {
        match self {
            ImageProcess::Threshold => "Black & white",
            ImageProcess::Dither => "Dither",
            ImageProcess::Grayscale => "Grayscale",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct ImageSrc {
    pub path: String,
    /// 0 = derive from the other dimension (or from `dpi` when both are 0).
    pub width_mm: f64,
    pub height_mm: f64,
    pub dpi: f64,
    pub process: ImageProcess,
    pub threshold: u8,
    /// -100..100
    pub brightness: f64,
    /// -100..100
    pub contrast: f64,
    /// Grayscale tones (2..16).
    pub levels: u8,
    pub invert: bool,
}

impl Default for ImageSrc {
    fn default() -> Self {
        Self {
            path: String::new(),
            width_mm: 30.0,
            height_mm: 0.0,
            dpi: 0.0,
            process: ImageProcess::Dither,
            threshold: 128,
            brightness: 0.0,
            contrast: 0.0,
            levels: 6,
            invert: false,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct SvgSrc {
    pub path: String,
    /// 0 = document size.
    pub width_mm: f64,
    pub threshold: u8,
    pub invert: bool,
    /// Vector mode: curve flattening tolerance.
    pub tolerance_mm: f64,
}

impl Default for SvgSrc {
    fn default() -> Self {
        Self { path: String::new(), width_mm: 0.0, threshold: 128, invert: false, tolerance_mm: 0.01 }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct QrSrc {
    pub data: String,
    pub module_mm: f64,
    /// L, M, Q or H.
    pub ecc: String,
}

impl Default for QrSrc {
    fn default() -> Self {
        Self { data: "https://example.com".into(), module_mm: 0.5, ecc: "M".into() }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct DxfSrc {
    pub path: String,
    pub scale: f64,
    /// Empty = all layers.
    pub layers: Vec<String>,
    pub tolerance_mm: f64,
}

impl Default for DxfSrc {
    fn default() -> Self {
        Self { path: String::new(), scale: 1.0, layers: Vec::new(), tolerance_mm: 0.01 }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ShapeKind {
    #[default]
    Rect,
    Circle,
    Cross,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct ShapeSrc {
    pub shape: ShapeKind,
    /// Circle: diameter.
    pub width_mm: f64,
    pub height_mm: f64,
}

impl Default for ShapeSrc {
    fn default() -> Self {
        Self { shape: ShapeKind::Rect, width_mm: 10.0, height_mm: 10.0 }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RulerSrc {
    pub length_mm: f64,
    pub step_mm: f64,
    pub labels: bool,
    pub font: String,
    pub label_mm: f64,
}

impl Default for RulerSrc {
    fn default() -> Self {
        Self { length_mm: 50.0, step_mm: 1.0, labels: true, font: "DejaVu Sans".into(), label_mm: 1.8 }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "lowercase")]
pub enum Source {
    Text(TextSrc),
    Image(ImageSrc),
    Svg(SvgSrc),
    Qr(QrSrc),
    Dxf(DxfSrc),
    Shape(ShapeSrc),
    Ruler(RulerSrc),
}

impl Source {
    pub const KINDS: [&'static str; 7] = ["text", "image", "svg", "qr", "dxf", "shape", "ruler"];

    pub fn default_of(kind: &str) -> Option<Source> {
        Some(match kind {
            "text" => Source::Text(TextSrc::default()),
            "image" => Source::Image(ImageSrc::default()),
            "svg" => Source::Svg(SvgSrc::default()),
            "qr" => Source::Qr(QrSrc::default()),
            "dxf" => Source::Dxf(DxfSrc::default()),
            "shape" => Source::Shape(ShapeSrc::default()),
            "ruler" => Source::Ruler(RulerSrc::default()),
            _ => return None,
        })
    }

    pub fn kind(&self) -> &'static str {
        match self {
            Source::Text(_) => "text",
            Source::Image(_) => "image",
            Source::Svg(_) => "svg",
            Source::Qr(_) => "qr",
            Source::Dxf(_) => "dxf",
            Source::Shape(_) => "shape",
            Source::Ruler(_) => "ruler",
        }
    }

    /// Supported modes, preferred first.
    pub fn modes(&self) -> &'static [Mode] {
        match self {
            Source::Text(_) => &[Mode::Raster, Mode::Vector],
            Source::Image(_) | Source::Qr(_) => &[Mode::Raster],
            Source::Svg(_) => &[Mode::Raster, Mode::Vector],
            Source::Dxf(_) | Source::Ruler(_) => &[Mode::Vector],
            Source::Shape(_) => &[Mode::Vector, Mode::Raster],
        }
    }

    /// Name shown to people.
    pub fn title(&self) -> &'static str {
        match self {
            Source::Text(_) => "Text",
            Source::Image(_) => "Image",
            Source::Svg(_) => "SVG",
            Source::Qr(_) => "QR code",
            Source::Dxf(_) => "DXF",
            Source::Shape(s) => match s.shape {
                ShapeKind::Rect => "Rectangle",
                ShapeKind::Circle => "Circle",
                ShapeKind::Cross => "Cross",
            },
            Source::Ruler(_) => "Ruler",
        }
    }

    /// Whether width and height can only scale together.
    pub fn keeps_aspect(&self) -> bool {
        match self {
            Source::Image(_) => false,
            Source::Shape(s) => s.shape == ShapeKind::Circle,
            _ => true,
        }
    }

    /// Resize by (fx, fy), given the current artwork size in mm (source axes).
    pub fn scale(&mut self, fx: f64, fy: f64, size_mm: (f64, f64), base_dir: &Path) -> Result<()> {
        let fy = if self.keeps_aspect() { fx } else { fy };
        let r = |v: f64| (v * 1e4).round() / 1e4;
        match self {
            Source::Image(s) => {
                s.width_mm = r(size_mm.0 * fx);
                s.height_mm = r(size_mm.1 * fy);
            }
            Source::Svg(s) => {
                let w = if s.width_mm > 0.0 { s.width_mm } else { svg_width_mm(&resolve(&s.path, base_dir))? };
                s.width_mm = r(w * fx);
            }
            Source::Dxf(d) => d.scale = (d.scale * fx * 1e6).round() / 1e6,
            Source::Text(t) => t.size_mm = r(t.size_mm * fx),
            Source::Qr(q) => q.module_mm = r((q.module_mm * fx).max(PITCH_MM)),
            Source::Shape(s) => {
                s.width_mm = r(s.width_mm * fx);
                s.height_mm = r(s.height_mm * fy);
            }
            Source::Ruler(rl) => rl.length_mm = r(rl.length_mm * fx),
        }
        Ok(())
    }

    pub fn path(&self) -> Option<&str> {
        match self {
            Source::Image(s) => Some(&s.path),
            Source::Svg(s) => Some(&s.path),
            Source::Dxf(s) => Some(&s.path),
            _ => None,
        }
    }

    /// Replace `${name}` in user strings.
    pub fn substituted(&self, vars: &std::collections::BTreeMap<String, String>) -> Source {
        if vars.is_empty() {
            return self.clone();
        }
        let sub = |s: &str| substitute(s, vars);
        let mut out = self.clone();
        match &mut out {
            Source::Text(t) => t.text = sub(&t.text),
            Source::Qr(q) => q.data = sub(&q.data),
            Source::Image(s) => s.path = sub(&s.path),
            Source::Svg(s) => s.path = sub(&s.path),
            Source::Dxf(s) => s.path = sub(&s.path),
            Source::Shape(_) | Source::Ruler(_) => {}
        }
        out
    }
}

/// `${name}` substitution; unknown names are left untouched.
pub fn substitute(s: &str, vars: &std::collections::BTreeMap<String, String>) -> String {
    let mut out = String::with_capacity(s.len());
    let mut rest = s;
    while let Some(i) = rest.find("${") {
        out.push_str(&rest[..i]);
        let tail = &rest[i + 2..];
        match tail.find('}') {
            Some(j) if vars.contains_key(&tail[..j]) => {
                out.push_str(&vars[&tail[..j]]);
                rest = &tail[j + 1..];
            }
            _ => {
                out.push_str("${");
                rest = tail;
            }
        }
    }
    out.push_str(rest);
    out
}

/// Rendered artwork.
#[derive(Debug, Clone)]
pub enum Artwork {
    Raster(Bitmap),
    /// Grayscale tones, lightest first; same size, disjoint pixels.
    Levels(Vec<Bitmap>),
    /// Polylines in mm, Y down, bounding box at the origin.
    Vector(Vec<Polyline>),
}

#[derive(Debug, Clone)]
pub struct Rendered {
    pub art: Artwork,
    pub notes: Vec<String>,
}

impl Rendered {
    fn raster(bm: Bitmap) -> Self {
        Self { art: Artwork::Raster(bm), notes: Vec::new() }
    }
    fn vector(p: Vec<Polyline>) -> Self {
        Self { art: Artwork::Vector(normalize(p)), notes: Vec::new() }
    }
    fn note(mut self, n: impl Into<String>) -> Self {
        self.notes.push(n.into());
        self
    }
}

pub fn resolve(path: &str, base_dir: &Path) -> PathBuf {
    let p = if let Some(rest) = path.strip_prefix("~/") {
        std::env::var_os("HOME").map(|h| PathBuf::from(h).join(rest)).unwrap_or_else(|| PathBuf::from(path))
    } else {
        PathBuf::from(path)
    };
    if p.is_absolute() { p } else { base_dir.join(p) }
}

fn normalize(polylines: Vec<Polyline>) -> Vec<Polyline> {
    let pts = polylines.iter().flatten();
    let x0 = pts.clone().map(|p| p.0).fold(f64::INFINITY, f64::min);
    let y0 = pts.map(|p| p.1).fold(f64::INFINITY, f64::min);
    if !x0.is_finite() {
        return Vec::new();
    }
    polylines.into_iter().map(|pl| pl.into_iter().map(|(x, y)| (x - x0, y - y0)).collect()).collect()
}

pub fn render(src: &Source, mode: Mode, base_dir: &Path) -> Result<Rendered> {
    if !src.modes().contains(&mode) {
        let modes: Vec<&str> = src.modes().iter().map(|m| m.as_str()).collect();
        bail!("{} supports {} mode only", src.kind(), modes.join(", "));
    }
    match (src, mode) {
        (Source::Text(t), Mode::Raster) => text_raster(t),
        (Source::Text(t), Mode::Vector) => {
            let (p, notes) = text_polylines(t)?;
            let mut r = Rendered::vector(p).note("glyph outlines; raster mode fills strokes");
            r.notes.extend(notes);
            Ok(r)
        }
        (Source::Image(s), _) => image(s, base_dir),
        (Source::Svg(s), Mode::Raster) => svg_raster(s, base_dir),
        (Source::Svg(s), Mode::Vector) => svg_vector(s, base_dir),
        (Source::Qr(q), _) => qr(q),
        (Source::Dxf(d), _) => dxf(d, base_dir),
        (Source::Shape(s), Mode::Raster) => shape_raster(s),
        // exact geometry from its own origin: the true outline's box sits at (x_mm, y_mm),
        // not the box of its flattened vertices
        (Source::Shape(s), Mode::Vector) => {
            Ok(Rendered { art: Artwork::Vector(shape_polylines(s)), notes: Vec::new() })
        }
        (Source::Ruler(r), _) => ruler(r),
    }
}

/// Modification time of a file-backed source, for cache invalidation.
pub fn source_mtime(src: &Source, base_dir: &Path) -> Option<std::time::SystemTime> {
    std::fs::metadata(resolve(src.path()?, base_dir)).and_then(|m| m.modified()).ok()
}

// ------------------------------------------------------------- fonts / svg

/// Shared system font database (loaded once).
pub fn fontdb() -> Arc<fontdb::Database> {
    static DB: OnceLock<Arc<fontdb::Database>> = OnceLock::new();
    DB.get_or_init(|| {
        let mut db = fontdb::Database::new();
        db.load_system_fonts();
        // fontdb's generic defaults are Windows families; map them to what exists here
        let pick =
            |db: &fontdb::Database, cands: &[&str]| cands.iter().find(|f| has_family(db, f)).map(|f| f.to_string());
        if let Some(f) = pick(&db, &["DejaVu Sans", "Liberation Sans", "Noto Sans", "Cantarell", "FreeSans"]) {
            db.set_sans_serif_family(f);
        }
        if let Some(f) = pick(&db, &["DejaVu Serif", "Liberation Serif", "Noto Serif", "FreeSerif"]) {
            db.set_serif_family(f);
        }
        if let Some(f) = pick(&db, &["DejaVu Sans Mono", "Liberation Mono", "Noto Sans Mono", "FreeMono"]) {
            db.set_monospace_family(f);
        }
        Arc::new(db)
    })
    .clone()
}

fn has_family(db: &fontdb::Database, name: &str) -> bool {
    db.faces().any(|f| f.families.iter().any(|(n, _)| n.eq_ignore_ascii_case(name)))
}

/// Installed font families, sorted, for UI pickers.
pub fn font_families() -> &'static [String] {
    static FAMILIES: OnceLock<Vec<String>> = OnceLock::new();
    FAMILIES.get_or_init(|| {
        let db = fontdb();
        let mut v: Vec<String> = db.faces().filter_map(|f| f.families.first().map(|(n, _)| n.clone())).collect();
        v.sort();
        v.dedup();
        v
    })
}

/// `dpi` converts absolute units (mm, in, pt) to user units.
fn svg_options(resources_dir: Option<PathBuf>, dpi: f32) -> usvg::Options<'static> {
    let mut opt = usvg::Options { fontdb: fontdb(), dpi, resources_dir, ..Default::default() };
    opt.font_family = "sans-serif".into();
    opt
}

struct FontSpec {
    family: String,
    weight: u16,
    italic: bool,
}

fn parse_font(spec: &str) -> FontSpec {
    let mut words: Vec<&str> = spec.split_whitespace().collect();
    let (mut weight, mut italic) = (400, false);
    while let Some(last) = words.last().map(|w| w.to_ascii_lowercase()) {
        match last.as_str() {
            "bold" => weight = 700,
            "semibold" | "demibold" => weight = 600,
            "medium" => weight = 500,
            "light" => weight = 300,
            "thin" => weight = 100,
            "black" | "heavy" => weight = 900,
            "regular" | "book" | "normal" => {}
            "italic" | "oblique" => italic = true,
            _ => break,
        }
        words.pop();
    }
    let family = if words.is_empty() { "sans-serif".to_string() } else { words.join(" ") };
    FontSpec { family, weight, italic }
}

fn xml_escape(s: &str) -> String {
    s.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;").replace('"', "&quot;")
}

/// Text laid out by usvg (glyphs become paths), in machine px.
fn text_tree(t: &TextSrc) -> Result<(usvg::Tree, Vec<String>)> {
    if !t.size_mm.is_finite() || t.size_mm <= 0.0 {
        bail!("size_mm must be > 0");
    }
    let f = parse_font(&t.font);
    let mut notes = Vec::new();
    let db = fontdb();
    if f.family != "sans-serif" && !has_family(&db, &f.family) {
        notes.push(format!("font “{}” not installed, using the default sans-serif", f.family));
    }
    let size = t.size_mm * PX_PER_MM;
    let anchor = match t.align {
        Align::Left => "start",
        Align::Center => "middle",
        Align::Right => "end",
    };
    let mut body = String::new();
    for (i, line) in t.text.lines().enumerate() {
        body += &format!("<text x=\"0\" y=\"{}\">{}</text>", i as f64 * size * 1.2, xml_escape(line));
    }
    let svg = format!(
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"10\" height=\"10\" xml:space=\"preserve\" \
         font-family=\"'{}', sans-serif\" font-size=\"{size}\" font-weight=\"{}\" font-style=\"{}\" \
         text-anchor=\"{anchor}\">{body}</svg>",
        xml_escape(&f.family),
        f.weight,
        if f.italic { "italic" } else { "normal" }
    );
    let tree = usvg::Tree::from_str(&svg, &svg_options(None, (25.4 * PX_PER_MM) as f32)).context("text layout")?;
    Ok((tree, notes))
}

/// Rasterise a whole tree (already in machine px) with its ink box at the origin.
fn rasterize_ink(tree: &usvg::Tree, scale: f32, threshold: u8, invert: bool, fit_to_ink: bool) -> Result<Bitmap> {
    let (ox, oy, w, h) = if fit_to_ink {
        let bb = tree.root().abs_stroke_bounding_box();
        if bb.width() <= 0.0 && bb.height() <= 0.0 {
            return Ok(Bitmap::new(0, 0));
        }
        (bb.x() * scale - 2.0, bb.y() * scale - 2.0, bb.width() * scale + 4.0, bb.height() * scale + 4.0)
    } else {
        let s = tree.size();
        (0.0, 0.0, s.width() * scale, s.height() * scale)
    };
    let (w, h) = (w.ceil().max(1.0) as u32, h.ceil().max(1.0) as u32);
    if w as u64 * h as u64 > 400_000_000 {
        bail!("artwork would be {w}x{h} px; check its size");
    }
    let mut pm = tiny_skia::Pixmap::new(w, h).ok_or_else(|| anyhow!("cannot allocate {w}x{h} px"))?;
    pm.fill(tiny_skia::Color::WHITE);
    let ts = tiny_skia::Transform::from_translate(-ox, -oy).pre_scale(scale, scale);
    resvg::render(tree, ts, &mut pm.as_mut());
    let px = pm.pixels();
    Ok(Bitmap::from_fn(w as usize, h as usize, |x, y| {
        let c = px[y * w as usize + x].demultiply();
        let luma = 0.299 * c.red() as f32 + 0.587 * c.green() as f32 + 0.114 * c.blue() as f32;
        (luma < threshold as f32) != invert
    }))
}

fn flatten_tree(group: &usvg::Group, scale: f64, tol_px: f64, out: &mut Vec<Polyline>, images: &mut usize) {
    for node in group.children() {
        match node {
            usvg::Node::Group(g) => flatten_tree(g, scale, tol_px, out, images),
            usvg::Node::Text(t) => flatten_tree(t.flattened(), scale, tol_px, out, images),
            usvg::Node::Image(_) => *images += 1,
            usvg::Node::Path(p) => {
                if p.is_visible() {
                    flatten_path(p.data(), p.abs_transform(), scale, tol_px, out);
                }
            }
        }
    }
}

/// Flatten curves with Wang's formula; returns polylines in px * scale.
fn flatten_path(path: &tiny_skia::Path, ts: tiny_skia::Transform, scale: f64, tol_px: f64, out: &mut Vec<Polyline>) {
    use tiny_skia::PathSegment as S;
    let map = |p: tiny_skia::Point| {
        let (x, y) = (p.x as f64, p.y as f64);
        (
            (ts.sx as f64 * x + ts.kx as f64 * y + ts.tx as f64) * scale,
            (ts.ky as f64 * x + ts.sy as f64 * y + ts.ty as f64) * scale,
        )
    };
    let tol = tol_px * scale;
    let mut cur: Polyline = Vec::new();
    let mut start = (0.0, 0.0);
    let flush = |cur: &mut Polyline, out: &mut Vec<Polyline>| {
        if cur.len() >= 2 {
            out.push(std::mem::take(cur));
        } else {
            cur.clear();
        }
    };
    for seg in path.segments() {
        match seg {
            S::MoveTo(p) => {
                flush(&mut cur, out);
                start = map(p);
                cur.push(start);
            }
            S::LineTo(p) => cur.push(map(p)),
            S::QuadTo(c, p) => {
                let p0 = *cur.last().unwrap_or(&start);
                let (c, p) = (map(c), map(p));
                let dd = ((p0.0 - 2.0 * c.0 + p.0).hypot(p0.1 - 2.0 * c.1 + p.1)) * 0.25;
                let n = ((dd / tol).sqrt().ceil() as usize).clamp(1, 512);
                for k in 1..=n {
                    let t = k as f64 / n as f64;
                    let u = 1.0 - t;
                    cur.push((
                        u * u * p0.0 + 2.0 * u * t * c.0 + t * t * p.0,
                        u * u * p0.1 + 2.0 * u * t * c.1 + t * t * p.1,
                    ));
                }
            }
            S::CubicTo(c1, c2, p) => {
                let p0 = *cur.last().unwrap_or(&start);
                let (c1, c2, p) = (map(c1), map(c2), map(p));
                let d1 = (p0.0 - 2.0 * c1.0 + c2.0).hypot(p0.1 - 2.0 * c1.1 + c2.1);
                let d2 = (c1.0 - 2.0 * c2.0 + p.0).hypot(c1.1 - 2.0 * c2.1 + p.1);
                let n = (((d1.max(d2) * 0.75) / tol).sqrt().ceil() as usize).clamp(1, 1024);
                for k in 1..=n {
                    let t = k as f64 / n as f64;
                    let u = 1.0 - t;
                    let (a, b, c, d) = (u * u * u, 3.0 * u * u * t, 3.0 * u * t * t, t * t * t);
                    cur.push((a * p0.0 + b * c1.0 + c * c2.0 + d * p.0, a * p0.1 + b * c1.1 + c * c2.1 + d * p.1));
                }
            }
            S::Close => {
                if !cur.is_empty() && cur.last() != Some(&start) {
                    cur.push(start);
                }
                flush(&mut cur, out);
            }
        }
    }
    flush(&mut cur, out);
}

// ---------------------------------------------------------------- text

fn text_raster(t: &TextSrc) -> Result<Rendered> {
    let (tree, notes) = text_tree(t)?;
    let bm = rasterize_ink(&tree, 1.0, 128, false, true)?;
    let bm = bm.crop_to_ink().map(|(_, b)| b).unwrap_or_else(|| Bitmap::new(0, 0));
    Ok(Rendered { art: Artwork::Raster(bm), notes })
}

/// Glyph outlines as closed polylines in mm (not normalised).
pub fn text_polylines(t: &TextSrc) -> Result<(Vec<Polyline>, Vec<String>)> {
    let (tree, notes) = text_tree(t)?;
    let mut out = Vec::new();
    let mut images = 0;
    flatten_tree(tree.root(), PITCH_MM, 0.1, &mut out, &mut images);
    Ok((out, notes))
}

// ---------------------------------------------------------------- svg

/// SVG user units are CSS px (1/96 in), as in browsers and Inkscape.
const SVG_PX_MM: f64 = 25.4 / 96.0;

fn parse_svg(path: &Path) -> Result<usvg::Tree> {
    let data = std::fs::read(path).with_context(|| format!("{}", path.display()))?;
    usvg::Tree::from_data(&data, &svg_options(path.parent().map(Path::to_path_buf), 96.0))
        .with_context(|| format!("{}", path.display()))
}

/// Document width of an SVG file, mm.
pub fn svg_width_mm(path: &Path) -> Result<f64> {
    Ok(parse_svg(path)?.size().width() as f64 * SVG_PX_MM)
}

/// Tree and its scale from user units to machine px.
fn load_svg(s: &SvgSrc, base_dir: &Path) -> Result<(usvg::Tree, f64)> {
    let tree = parse_svg(&resolve(&s.path, base_dir))?;
    let w = tree.size().width() as f64;
    let scale = if s.width_mm > 0.0 { mm2px(s.width_mm) as f64 / w } else { SVG_PX_MM * PX_PER_MM };
    Ok((tree, scale))
}

fn svg_raster(s: &SvgSrc, base_dir: &Path) -> Result<Rendered> {
    let (tree, scale) = load_svg(s, base_dir)?;
    let bm = rasterize_ink(&tree, scale as f32, s.threshold, s.invert, false)?;
    let (w, h) = (bm.width as f64 * PITCH_MM, bm.height as f64 * PITCH_MM);
    Ok(Rendered::raster(bm).note(format!("document {w:.2} x {h:.2} mm")))
}

fn svg_vector(s: &SvgSrc, base_dir: &Path) -> Result<Rendered> {
    let (tree, scale) = load_svg(s, base_dir)?;
    let mut out = Vec::new();
    let mut images = 0;
    let tol_px = (s.tolerance_mm.max(0.001) * PX_PER_MM) / scale;
    flatten_tree(tree.root(), scale * PITCH_MM, tol_px, &mut out, &mut images);
    let mut r = Rendered::vector(out).note("vector mode follows path outlines; fills are not hatched");
    if images > 0 {
        r = r.note(format!("{images} embedded bitmap(s) ignored in vector mode"));
    }
    Ok(r)
}

// --------------------------------------------------------------- images

fn image(s: &ImageSrc, base_dir: &Path) -> Result<Rendered> {
    let path = resolve(&s.path, base_dir);
    let (iw, ih) = image_size(&path).with_context(|| format!("{}", path.display()))?;
    let (iw, ih) = (iw as f64, ih as f64);
    let mut notes = Vec::new();
    let (mut w_mm, mut h_mm) = (s.width_mm, s.height_mm);
    if w_mm <= 0.0 && h_mm <= 0.0 {
        let dpi = if s.dpi > 0.0 { s.dpi } else { 96.0 };
        w_mm = iw / dpi * 25.4;
        notes.push(format!("size from {dpi} dpi"));
    }
    if w_mm > 0.0 && h_mm <= 0.0 {
        h_mm = w_mm * ih / iw;
    } else if h_mm > 0.0 && w_mm <= 0.0 {
        w_mm = h_mm * iw / ih;
    }
    let (w, h) = (mm2px(w_mm).max(1) as u32, mm2px(h_mm).max(1) as u32);
    let planes = image_planes(&path, w, h)?;
    // darkness 0..1 after brightness, contrast and invert, masked by coverage
    let contrast = (1.0 + s.contrast.clamp(-100.0, 100.0) / 100.0) as f32;
    let brightness = (s.brightness.clamp(-100.0, 100.0) * 1.28) as f32;
    let dark: Vec<f32> = planes
        .luma
        .pixels()
        .zip(planes.alpha.pixels())
        .map(|(l, a)| {
            let v = ((l.0[0] as f32 - 128.0) * contrast + 128.0 + brightness).clamp(0.0, 255.0);
            let d = 1.0 - v / 255.0;
            (if s.invert { 1.0 - d } else { d }) * (a.0[0] as f32 / 255.0)
        })
        .collect();
    let (wu, hu) = (w as usize, h as usize);
    let art = match s.process {
        ImageProcess::Threshold => {
            let t = 1.0 - s.threshold as f32 / 255.0;
            Artwork::Raster(Bitmap::from_fn(wu, hu, |x, y| dark[y * wu + x] > t))
        }
        ImageProcess::Dither => Artwork::Raster(floyd_steinberg(wu, hu, &dark)),
        ImageProcess::Grayscale => {
            let n = s.levels.clamp(2, 16) as usize;
            let tone = |x: usize, y: usize| (dark[y * wu + x] * n as f32).round() as usize;
            notes.push(format!("{n} tones: one pass per tone, power scaled from min to max"));
            Artwork::Levels((1..=n).map(|k| Bitmap::from_fn(wu, hu, |x, y| tone(x, y) == k)).collect())
        }
    };
    notes.push(format!("{w_mm:.2} x {h_mm:.2} mm from {iw}x{ih} px"));
    Ok(Rendered { art, notes })
}

/// Luma composited on white, and coverage, both resized to the burn size.
struct Planes {
    luma: image::GrayImage,
    alpha: image::GrayImage,
}

/// Decoding and resampling dominate image rendering; tone settings change far
/// more often than the file or the size, so the planes are kept for reuse.
fn image_planes(path: &Path, w: u32, h: u32) -> Result<Arc<Planes>> {
    use image::{ImageDecoder, ImageReader, imageops};
    type Key = (PathBuf, Option<std::time::SystemTime>, u32, u32);
    static CACHE: std::sync::Mutex<Vec<(Key, Arc<Planes>)>> = std::sync::Mutex::new(Vec::new());
    let mtime = std::fs::metadata(path).and_then(|m| m.modified()).ok();
    let key = (path.to_path_buf(), mtime, w, h);
    if let Some(p) = CACHE.lock().unwrap().iter().find(|(k, _)| *k == key).map(|(_, p)| p.clone()) {
        return Ok(p);
    }
    let mut decoder = ImageReader::open(path)
        .with_context(|| format!("{}", path.display()))?
        .with_guessed_format()?
        .into_decoder()
        .with_context(|| format!("{}", path.display()))?;
    let orientation = decoder.orientation().ok();
    let mut img = image::DynamicImage::from_decoder(decoder)?;
    if let Some(o) = orientation {
        img.apply_orientation(o);
    }
    // transparent areas never burn, even inverted
    let rgba = img.to_rgba8();
    let luma = image::GrayImage::from_fn(rgba.width(), rgba.height(), |x, y| {
        let p = rgba.get_pixel(x, y).0;
        let a = p[3] as f32 / 255.0;
        let l = 0.299 * p[0] as f32 + 0.587 * p[1] as f32 + 0.114 * p[2] as f32;
        image::Luma([(l * a + 255.0 * (1.0 - a)).round() as u8])
    });
    let alpha = image::GrayImage::from_fn(rgba.width(), rgba.height(), |x, y| image::Luma([rgba.get_pixel(x, y).0[3]]));
    let planes = Arc::new(Planes {
        luma: imageops::resize(&luma, w, h, imageops::FilterType::Lanczos3),
        alpha: imageops::resize(&alpha, w, h, imageops::FilterType::Triangle),
    });
    let mut cache = CACHE.lock().unwrap();
    cache.push((key, planes.clone()));
    if cache.len() > 4 {
        cache.remove(0);
    }
    Ok(planes)
}

/// Pixel size of an image file, after EXIF orientation.
pub fn image_size(path: &Path) -> Result<(u32, u32)> {
    use image::metadata::Orientation as O;
    use image::{ImageDecoder, ImageReader};
    let mut decoder = ImageReader::open(path)?.with_guessed_format()?.into_decoder()?;
    let (w, h) = decoder.dimensions();
    let turned = matches!(decoder.orientation(), Ok(O::Rotate90 | O::Rotate270 | O::Rotate90FlipH | O::Rotate270FlipH));
    Ok(if turned { (h, w) } else { (w, h) })
}

/// Error diffusion on darkness values (0 = white, 1 = burn).
fn floyd_steinberg(w: usize, h: usize, dark: &[f32]) -> Bitmap {
    let mut buf = dark.to_vec();
    let mut bm = Bitmap::new(w, h);
    for y in 0..h {
        for x in 0..w {
            let old = buf[y * w + x];
            let burn = old >= 0.5;
            bm.set(x, y, burn);
            let err = old - if burn { 1.0 } else { 0.0 };
            let mut spread = |dx: isize, dy: usize, f: f32| {
                let xx = x as isize + dx;
                if xx >= 0 && (xx as usize) < w && y + dy < h {
                    buf[(y + dy) * w + xx as usize] += err * f;
                }
            };
            spread(1, 0, 7.0 / 16.0);
            spread(-1, 1, 3.0 / 16.0);
            spread(0, 1, 5.0 / 16.0);
            spread(1, 1, 1.0 / 16.0);
        }
    }
    bm
}

// ------------------------------------------------------------------ qr

fn qr(q: &QrSrc) -> Result<Rendered> {
    use qrcode::{Color, EcLevel, QrCode};
    let ecc = match q.ecc.to_ascii_uppercase().as_str() {
        "L" => EcLevel::L,
        "M" => EcLevel::M,
        "Q" => EcLevel::Q,
        "H" => EcLevel::H,
        other => bail!("ecc must be L, M, Q or H, not {other:?}"),
    };
    let code = QrCode::with_error_correction_level(q.data.as_bytes(), ecc).map_err(|e| anyhow!("QR: {e}"))?;
    let n = code.width();
    let colors = code.to_colors();
    let m = mm2px(q.module_mm).max(1) as usize;
    let bm = Bitmap::from_fn(n * m, n * m, |x, y| colors[(y / m) * n + x / m] == Color::Dark);
    let note = format!("{n}x{n} modules of {:.2} mm; keep a 4-module quiet zone clear", m as f64 * PITCH_MM);
    Ok(Rendered::raster(bm).note(note))
}

// ------------------------------------------------------------ geometry

fn arc(cx: f64, cy: f64, r: f64, a0: f64, a1: f64, tol: f64) -> Polyline {
    let step = if r > tol { 2.0 * (1.0 - tol / r).max(-1.0).acos() } else { a1 - a0 };
    let n = (((a1 - a0).abs() / step).ceil() as usize).max(8);
    (0..=n)
        .map(|k| {
            let a = a0 + (a1 - a0) * k as f64 / n as f64;
            (cx + r * a.cos(), cy + r * a.sin())
        })
        .collect()
}

fn shape_polylines(s: &ShapeSrc) -> Vec<Polyline> {
    let (w, h) = (s.width_mm, s.height_mm);
    match s.shape {
        ShapeKind::Rect => vec![vec![(0.0, 0.0), (w, 0.0), (w, h), (0.0, h), (0.0, 0.0)]],
        ShapeKind::Circle => vec![arc(w / 2.0, w / 2.0, w / 2.0, 0.0, std::f64::consts::TAU, 0.01)],
        ShapeKind::Cross => vec![vec![(0.0, h / 2.0), (w, h / 2.0)], vec![(w / 2.0, 0.0), (w / 2.0, h)]],
    }
}

fn shape_raster(s: &ShapeSrc) -> Result<Rendered> {
    let (w, h) = (mm2px(s.width_mm).max(1) as usize, mm2px(s.height_mm).max(1) as usize);
    let bm = match s.shape {
        ShapeKind::Rect => Bitmap::filled(w, h),
        ShapeKind::Circle => {
            let r = w as f64 / 2.0;
            Bitmap::from_fn(w, w, |x, y| (x as f64 + 0.5 - r).powi(2) + (y as f64 + 0.5 - r).powi(2) <= r * r)
        }
        ShapeKind::Cross => Bitmap::from_fn(w, h, |x, y| y == h / 2 || x == w / 2),
    };
    Ok(Rendered::raster(bm))
}

fn ruler(r: &RulerSrc) -> Result<Rendered> {
    if !(r.step_mm > 0.0 && r.length_mm > 0.0) {
        bail!("length_mm and step_mm must be > 0");
    }
    let mut polys = vec![vec![(0.0, 0.0), (r.length_mm, 0.0)]];
    let n = (r.length_mm / r.step_mm).round() as usize;
    let near = |v: f64| (v - v.round()).abs() < 1e-9;
    for k in 0..=n {
        let x = k as f64 * r.step_mm;
        let major = near(x / 10.0);
        let tick = if major {
            3.0
        } else if near(x / 5.0) {
            2.0
        } else {
            1.2
        };
        polys.push(vec![(x, 0.0), (x, tick)]);
        if major && r.labels {
            let label = TextSrc { text: format!("{x}"), font: r.font.clone(), size_mm: r.label_mm, align: Align::Left };
            let (glyphs, _) = text_polylines(&label)?;
            let glyphs = normalize(glyphs);
            let gw = glyphs.iter().flatten().map(|p| p.0).fold(0.0, f64::max);
            polys.extend(
                glyphs.into_iter().map(|pl| pl.into_iter().map(|(px, py)| (px + x - gw / 2.0, py + 3.6)).collect()),
            );
        }
    }
    // not normalised: tick 0 sits exactly at (x_mm, y_mm); labels may extend left of it
    Ok(Rendered {
        art: Artwork::Vector(polys),
        notes: vec![format!("tick 0 at the op position; verify {} mm with calipers", r.length_mm)],
    })
}

fn dxf(d: &DxfSrc, base_dir: &Path) -> Result<Rendered> {
    let path = resolve(&d.path, base_dir);
    let layers = (!d.layers.is_empty()).then_some(d.layers.as_slice());
    let res = crate::dxf::read_dxf(&path, d.tolerance_mm, layers).with_context(|| format!("{}", path.display()))?;
    if res.polylines.is_empty() {
        bail!("DXF contains no supported geometry");
    }
    let y_max = res.polylines.iter().flatten().map(|p| p.1).fold(f64::NEG_INFINITY, f64::max);
    let polys = res
        .polylines
        .into_iter()
        .map(|pl| pl.into_iter().map(|(x, y)| (x * d.scale, (y_max - y) * d.scale)).collect())
        .collect();
    let mut r = Rendered::vector(polys).note(format!("units: {}", res.units));
    if !res.skipped.is_empty() {
        let s: Vec<String> = res.skipped.iter().map(|(k, v)| format!("{k}x{v}")).collect();
        r = r.note(format!("skipped: {}", s.join(", ")));
    }
    Ok(r)
}
