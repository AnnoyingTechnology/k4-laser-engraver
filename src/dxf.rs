//! Minimal ASCII DXF reader producing polylines in millimetres (Y up).
//!
//! Supported: LINE, LWPOLYLINE/POLYLINE (with bulges), CIRCLE, ARC, ELLIPSE,
//! SPLINE (NURBS via de Boor, or fit points), INSERT (block expansion with
//! scale/rotation, nested). OCS extrusion (0,0,-1) mirroring is honoured for
//! 2-D entities. TEXT/MTEXT/DIMENSION/HATCH are reported as skipped.

use std::collections::{BTreeMap, HashMap};
use std::f64::consts::{PI, TAU};
use std::path::Path;

use anyhow::{Result, bail};

use crate::sources::Polyline;

/// `$INSUNITS` code -> millimetres (0 = unitless, assumed mm).
fn unit_to_mm(code: i64) -> f64 {
    match code {
        1 => 25.4,
        2 => 304.8,
        5 => 10.0,
        6 => 1000.0,
        8 => 2.54e-5,
        9 => 0.0254,
        10 => 914.4,
        13 => 1e-3,
        14 => 100.0,
        _ => 1.0,
    }
}

fn unit_name(code: i64) -> String {
    match code {
        0 => "unitless (assumed mm)".into(),
        1 => "inch".into(),
        2 => "foot".into(),
        4 => "mm".into(),
        5 => "cm".into(),
        6 => "m".into(),
        _ => format!("INSUNITS={code}"),
    }
}

#[derive(Debug, Default, Clone)]
struct Entity {
    kind: String,
    tags: Vec<(i32, String)>,
    vertices: Vec<Entity>,
}

impl Entity {
    fn new(kind: &str) -> Self {
        Self { kind: kind.to_string(), ..Default::default() }
    }

    fn f(&self, code: i32, default: f64) -> f64 {
        self.tags.iter().find(|(c, _)| *c == code).and_then(|(_, v)| v.trim().parse().ok()).unwrap_or(default)
    }

    fn i(&self, code: i32) -> i64 {
        self.tags.iter().find(|(c, _)| *c == code).and_then(|(_, v)| v.trim().parse().ok()).unwrap_or(0)
    }

    fn s(&self, code: i32, default: &str) -> String {
        self.tags.iter().find(|(c, _)| *c == code).map(|(_, v)| v.clone()).unwrap_or_else(|| default.to_string())
    }

    fn all(&self, code: i32) -> Vec<f64> {
        self.tags.iter().filter(|(c, _)| *c == code).filter_map(|(_, v)| v.trim().parse().ok()).collect()
    }

    fn layer(&self) -> String {
        self.s(8, "0")
    }
}

struct Block {
    base: (f64, f64),
    entities: Vec<Entity>,
}

#[derive(Debug)]
pub struct DxfResult {
    /// Drawing space, mm, Y up.
    pub polylines: Vec<Polyline>,
    pub skipped: BTreeMap<String, usize>,
    pub units: String,
}

struct Parsed {
    header: HashMap<String, String>,
    blocks: HashMap<String, Block>,
    entities: Vec<Entity>,
}

fn parse(text: &str) -> Result<Parsed> {
    let lines: Vec<&str> = text.lines().collect();
    let mut pairs = Vec::with_capacity(lines.len() / 2);
    for c in lines.chunks_exact(2) {
        let code: i32 = match c[0].trim().parse() {
            Ok(v) => v,
            Err(_) => bail!("malformed group code {:?}", c[0].trim()),
        };
        pairs.push((code, c[1].trim().to_string()));
    }
    let mut p = Parsed { header: HashMap::new(), blocks: HashMap::new(), entities: Vec::new() };
    let mut section: Option<String> = None;
    let mut block: Option<(String, Block)> = None;
    let mut cur: Option<Entity> = None;
    let mut last_var: Option<String> = None;
    let mut polyline: Option<usize> = None; // index of a POLYLINE awaiting VERTEX/SEQEND
    let mut in_block_poly = false;
    let mut i = 0;
    while i < pairs.len() {
        let (code, value) = (pairs[i].0, pairs[i].1.clone());
        i += 1;
        if code == 0 {
            if let Some(e) = cur.take() {
                let in_entities = matches!(section.as_deref(), Some("ENTITIES") | Some("BLOCKS"));
                let target: &mut Vec<Entity> = match block.as_mut() {
                    Some((_, b)) => &mut b.entities,
                    None => &mut p.entities,
                };
                match e.kind.as_str() {
                    "VERTEX" if polyline.is_some() => target[polyline.unwrap()].vertices.push(e),
                    "SEQEND" => polyline = None,
                    "POLYLINE" => {
                        target.push(e);
                        polyline = Some(target.len() - 1);
                        in_block_poly = block.is_some();
                    }
                    "BLOCK" if section.as_deref() == Some("BLOCKS") => {
                        let name = e.s(2, "");
                        block = Some((name, Block { base: (e.f(10, 0.0), e.f(20, 0.0)), entities: Vec::new() }));
                    }
                    "ENDBLK" if section.as_deref() == Some("BLOCKS") => {
                        if let Some((name, b)) = block.take() {
                            p.blocks.insert(name, b);
                        }
                        if in_block_poly {
                            polyline = None;
                        }
                    }
                    _ if in_entities => target.push(e),
                    _ => {}
                }
            }
            match value.as_str() {
                "SECTION" => {
                    if let Some((_, name)) = pairs.get(i) {
                        section = Some(name.clone());
                        i += 1;
                    }
                }
                "ENDSEC" => section = None,
                "EOF" => break,
                kind if matches!(section.as_deref(), Some("ENTITIES") | Some("BLOCKS")) => {
                    cur = Some(Entity::new(kind))
                }
                _ => {}
            }
            continue;
        }
        if section.as_deref() == Some("HEADER") {
            if code == 9 {
                last_var = Some(value);
            } else if let Some(v) = &last_var {
                p.header.entry(v.clone()).or_insert(value);
            }
        } else if let Some(e) = cur.as_mut() {
            e.tags.push((code, value));
        }
    }
    Ok(p)
}

// ------------------------------------------------------------- geometry

/// CCW arc from a0 to a1 (radians).
fn arc_points(cx: f64, cy: f64, r: f64, a0: f64, mut a1: f64, tol: f64) -> Polyline {
    while a1 <= a0 {
        a1 += TAU;
    }
    let sweep = a1 - a0;
    let step = if r > tol { 2.0 * (1.0 - tol / r).clamp(-1.0, 1.0).acos() } else { sweep };
    let n = ((sweep / step.max(1e-6)).ceil() as usize).max(2);
    (0..=n)
        .map(|k| {
            let a = a0 + sweep * k as f64 / n as f64;
            (cx + r * a.cos(), cy + r * a.sin())
        })
        .collect()
}

fn bulge_points(p0: (f64, f64), p1: (f64, f64), bulge: f64, tol: f64) -> Polyline {
    if bulge.abs() < 1e-12 {
        return vec![p0, p1];
    }
    let ((x0, y0), (x1, y1)) = (p0, p1);
    let chord = (x1 - x0).hypot(y1 - y0);
    if chord == 0.0 {
        return vec![p0];
    }
    let theta = 4.0 * bulge.atan();
    let r = chord / (2.0 * (theta.abs() / 2.0).sin());
    let (mx, my) = ((x0 + x1) / 2.0, (y0 + y1) / 2.0);
    let d = (r * r - (chord / 2.0).powi(2)).max(0.0).sqrt();
    let (ux, uy) = ((x1 - x0) / chord, (y1 - y0) / chord);
    let sign = if bulge > 0.0 { 1.0 } else { -1.0 };
    // centre is left of the chord for CCW (positive bulge) when |theta| < pi
    let side = sign * if theta.abs() < PI { 1.0 } else { -1.0 };
    let (cx, cy) = (mx - side * d * uy, my + side * d * ux);
    let a0 = (y0 - cy).atan2(x0 - cx);
    let a1 = (y1 - cy).atan2(x1 - cx);
    if bulge > 0.0 {
        arc_points(cx, cy, r, a0, a1, tol)
    } else {
        let mut v = arc_points(cx, cy, r, a1, a0, tol);
        v.reverse();
        v
    }
}

fn de_boor(degree: usize, knots: &[f64], ctrl: &[(f64, f64)], weights: &[f64], t: f64) -> (f64, f64) {
    let n = ctrl.len() - 1;
    let k = degree;
    let mut span = k;
    while span < n && knots[span + 1] <= t {
        span += 1;
    }
    let mut d: Vec<[f64; 3]> =
        (span - k..=span).map(|j| [ctrl[j].0 * weights[j], ctrl[j].1 * weights[j], weights[j]]).collect();
    for r in 1..=k {
        for j in (r..=k).rev() {
            let i = j + span - k;
            let denom = knots[i + k - r + 1] - knots[i];
            let a = if denom == 0.0 { 0.0 } else { (t - knots[i]) / denom };
            let prev = d[j - 1];
            for (v, p) in d[j].iter_mut().zip(prev) {
                *v = (1.0 - a) * p + a * *v;
            }
        }
    }
    let [x, y, w] = d[k];
    (x / w, y / w)
}

fn spline_points(e: &Entity, tol: f64) -> Polyline {
    let degree = e.f(71, 3.0) as usize;
    let knots = e.all(40);
    let ctrl: Vec<(f64, f64)> = e.all(10).into_iter().zip(e.all(20)).collect();
    let mut weights = e.all(41);
    if weights.len() != ctrl.len() {
        weights = vec![1.0; ctrl.len()];
    }
    if degree >= 1 && ctrl.len() > degree && knots.len() == ctrl.len() + degree + 1 {
        let (t0, t1) = (knots[degree], knots[ctrl.len()]);
        let approx: f64 = ctrl.windows(2).map(|w| (w[1].0 - w[0].0).hypot(w[1].1 - w[0].1)).sum();
        let n = ((approx / (tol * 20.0).max(1e-3)) as usize).clamp(16, 4000);
        return (0..=n)
            .map(|s| de_boor(degree, &knots, &ctrl, &weights, t0 + (t1 - t0) * s as f64 / n as f64))
            .collect();
    }
    let fit: Polyline = e.all(11).into_iter().zip(e.all(21)).collect();
    if fit.len() >= 2 { fit } else { ctrl }
}

fn ocs(e: &Entity, pts: Polyline) -> Polyline {
    if e.f(230, 1.0) < 0.0 { pts.into_iter().map(|(x, y)| (-x, y)).collect() } else { pts }
}

fn poly_with_bulges(verts: &[(f64, f64)], bulges: &[f64], closed: bool, tol: f64) -> Polyline {
    if verts.is_empty() {
        return Vec::new();
    }
    let n = verts.len();
    let mut pts = vec![verts[0]];
    let last = if closed { n } else { n - 1 };
    for i in 0..last {
        let seg = bulge_points(verts[i], verts[(i + 1) % n], bulges[i], tol);
        pts.extend_from_slice(&seg[1..]);
    }
    pts
}

fn entity_polylines(e: &Entity, tol: f64) -> Option<Vec<Polyline>> {
    let pl = match e.kind.as_str() {
        "LINE" => vec![(e.f(10, 0.0), e.f(20, 0.0)), (e.f(11, 0.0), e.f(21, 0.0))],
        "LWPOLYLINE" => {
            let closed = e.i(70) & 1 == 1;
            let (mut verts, mut bulges): (Vec<(f64, f64)>, Vec<f64>) = (Vec::new(), Vec::new());
            for (c, v) in &e.tags {
                let v: f64 = v.trim().parse().unwrap_or(0.0);
                match c {
                    10 => {
                        verts.push((v, 0.0));
                        bulges.push(0.0);
                    }
                    20 if !verts.is_empty() => verts.last_mut().unwrap().1 = v,
                    42 if !verts.is_empty() => *bulges.last_mut().unwrap() = v,
                    _ => {}
                }
            }
            ocs(e, poly_with_bulges(&verts, &bulges, closed, tol))
        }
        "POLYLINE" => {
            let closed = e.i(70) & 1 == 1;
            let verts: Vec<(f64, f64)> = e.vertices.iter().map(|v| (v.f(10, 0.0), v.f(20, 0.0))).collect();
            let bulges: Vec<f64> = e.vertices.iter().map(|v| v.f(42, 0.0)).collect();
            ocs(e, poly_with_bulges(&verts, &bulges, closed, tol))
        }
        "CIRCLE" => ocs(e, arc_points(e.f(10, 0.0), e.f(20, 0.0), e.f(40, 0.0), 0.0, TAU, tol)),
        "ARC" => {
            let (a0, a1) = (e.f(50, 0.0).to_radians(), e.f(51, 360.0).to_radians());
            ocs(e, arc_points(e.f(10, 0.0), e.f(20, 0.0), e.f(40, 0.0), a0, a1, tol))
        }
        "ELLIPSE" => {
            let (cx, cy) = (e.f(10, 0.0), e.f(20, 0.0));
            let (mx, my) = (e.f(11, 1.0), e.f(21, 0.0));
            let ratio = e.f(40, 1.0);
            let (t0, mut t1) = (e.f(41, 0.0), e.f(42, TAU));
            while t1 <= t0 {
                t1 += TAU;
            }
            let a = mx.hypot(my);
            let rot = my.atan2(mx);
            let step = 2.0 * (1.0 - tol / a.max(tol)).max(-1.0).acos();
            let n = (((t1 - t0) / step.max(1e-6)).ceil() as usize).max(16);
            (0..=n)
                .map(|s| {
                    let t = t0 + (t1 - t0) * s as f64 / n as f64;
                    let (x, y) = (a * t.cos(), a * ratio * t.sin());
                    (cx + x * rot.cos() - y * rot.sin(), cy + x * rot.sin() + y * rot.cos())
                })
                .collect()
        }
        "SPLINE" => {
            let mut pts = spline_points(e, tol);
            if e.i(70) & 1 == 1 && !pts.is_empty() {
                pts.push(pts[0]);
            }
            pts
        }
        _ => return None,
    };
    Some(vec![pl])
}

fn transform(polys: Vec<Polyline>, base: (f64, f64), ins: (f64, f64), sx: f64, sy: f64, rot: f64) -> Vec<Polyline> {
    let (c, s) = (rot.cos(), rot.sin());
    polys
        .into_iter()
        .map(|pl| {
            pl.into_iter()
                .map(|(x, y)| {
                    let (x, y) = ((x - base.0) * sx, (y - base.1) * sy);
                    (x * c - y * s + ins.0, x * s + y * c + ins.1)
                })
                .collect()
        })
        .collect()
}

pub fn read_dxf(path: &Path, tolerance_mm: f64, layers: Option<&[String]>) -> Result<DxfResult> {
    let raw = std::fs::read(path)?;
    if raw.starts_with(b"AutoCAD Binary DXF") {
        bail!("binary DXF is not supported; save as ASCII DXF");
    }
    parse_dxf(&String::from_utf8_lossy(&raw), tolerance_mm, layers)
}

pub fn parse_dxf(text: &str, tolerance_mm: f64, layers: Option<&[String]>) -> Result<DxfResult> {
    let p = parse(text)?;
    let units: i64 = p.header.get("$INSUNITS").and_then(|v| v.trim().parse().ok()).unwrap_or(0);
    let to_mm = unit_to_mm(units);
    let tol = tolerance_mm.max(1e-4) / to_mm;
    let mut skipped: BTreeMap<String, usize> = BTreeMap::new();

    fn expand(
        ents: &[Entity],
        depth: usize,
        p: &Parsed,
        tol: f64,
        layers: Option<&[String]>,
        skipped: &mut BTreeMap<String, usize>,
    ) -> Vec<Polyline> {
        let mut out = Vec::new();
        for e in ents {
            if e.kind == "INSERT" {
                let Some(block) = p.blocks.get(&e.s(2, "")).filter(|_| depth <= 16) else {
                    *skipped.entry("INSERT(unresolved)".into()).or_default() += 1;
                    continue;
                };
                let inner = expand(&block.entities, depth + 1, p, tol, layers, skipped);
                out.extend(transform(
                    inner,
                    block.base,
                    (e.f(10, 0.0), e.f(20, 0.0)),
                    e.f(41, 1.0),
                    e.f(42, 1.0),
                    e.f(50, 0.0).to_radians(),
                ));
                continue;
            }
            if layers.is_some_and(|l| !l.contains(&e.layer())) {
                continue;
            }
            match entity_polylines(e, tol) {
                Some(polys) => out.extend(polys.into_iter().filter(|pl| pl.len() >= 2)),
                None => {
                    if !matches!(e.kind.as_str(), "POINT" | "VIEWPORT" | "ATTDEF" | "ATTRIB") {
                        *skipped.entry(e.kind.clone()).or_default() += 1;
                    }
                }
            }
        }
        out
    }

    let polylines = expand(&p.entities, 0, &p, tol, layers, &mut skipped)
        .into_iter()
        .map(|pl| pl.into_iter().map(|(x, y)| (x * to_mm, y * to_mm)).collect())
        .collect();
    Ok(DxfResult { polylines, skipped, units: unit_name(units) })
}

#[cfg(test)]
mod tests {
    use super::*;

    const DXF: &str = "0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n0\nSECTION\n2\nENTITIES\n\
0\nLINE\n8\n0\n10\n0\n20\n0\n11\n20\n21\n0\n\
0\nCIRCLE\n8\n0\n10\n10\n20\n10\n40\n5\n\
0\nLWPOLYLINE\n8\nCUT\n90\n2\n70\n0\n10\n0\n20\n20\n42\n1\n10\n10\n20\n20\n\
0\nTEXT\n8\n0\n1\nhello\n\
0\nENDSEC\n0\nEOF\n";

    #[test]
    fn entities_units_layers() {
        let res = parse_dxf(DXF, 0.01, None).unwrap();
        assert_eq!(res.units, "mm");
        assert_eq!(res.polylines.len(), 3);
        assert_eq!(res.skipped.get("TEXT"), Some(&1));
        // semicircle bulge from (0,20) to (10,20) CCW dips to y = 15
        let min_y = res.polylines[2].iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
        assert!((min_y - 15.0).abs() < 0.011, "{min_y}");
        let max_y = res.polylines.iter().flatten().map(|p| p.1).fold(f64::NEG_INFINITY, f64::max);
        assert!((max_y - 20.0).abs() < 1e-6);
        // circle radius respected within tolerance
        for &(x, y) in &res.polylines[1] {
            assert!(((x - 10.0).hypot(y - 10.0) - 5.0).abs() < 1e-9);
        }
        let only_cut = parse_dxf(DXF, 0.01, Some(&["CUT".to_string()])).unwrap();
        assert_eq!(only_cut.polylines.len(), 1);
    }

    #[test]
    fn inch_units_and_blocks() {
        let dxf = "0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n1\n0\nENDSEC\n\
0\nSECTION\n2\nBLOCKS\n0\nBLOCK\n2\nB1\n10\n0\n20\n0\n0\nLINE\n8\n0\n10\n0\n20\n0\n11\n1\n21\n0\n0\nENDBLK\n0\nENDSEC\n\
0\nSECTION\n2\nENTITIES\n0\nINSERT\n2\nB1\n10\n1\n20\n1\n50\n90\n0\nENDSEC\n0\nEOF\n";
        let res = parse_dxf(dxf, 0.01, None).unwrap();
        assert_eq!(res.units, "inch");
        let pl = &res.polylines[0];
        assert!((pl[0].0 - 25.4).abs() < 1e-9 && (pl[0].1 - 25.4).abs() < 1e-9);
        assert!((pl[1].0 - 25.4).abs() < 1e-9 && (pl[1].1 - 50.8).abs() < 1e-9);
    }
}
