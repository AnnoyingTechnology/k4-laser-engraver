//! Machine-level plan: operations already quantised to 0.05 mm pixels.

use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};
use std::sync::Arc;

use crate::protocol::{self as P, Segment};

/// Row-major boolean image, `true` = burn.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct Bitmap {
    pub width: usize,
    pub height: usize,
    pub data: Vec<bool>,
}

impl Bitmap {
    pub fn new(width: usize, height: usize) -> Self {
        Self { width, height, data: vec![false; width * height] }
    }

    pub fn filled(width: usize, height: usize) -> Self {
        Self { width, height, data: vec![true; width * height] }
    }

    pub fn from_fn(width: usize, height: usize, f: impl Fn(usize, usize) -> bool) -> Self {
        let mut data = Vec::with_capacity(width * height);
        for y in 0..height {
            for x in 0..width {
                data.push(f(x, y));
            }
        }
        Self { width, height, data }
    }

    #[inline]
    pub fn get(&self, x: usize, y: usize) -> bool {
        self.data[y * self.width + x]
    }

    #[inline]
    pub fn set(&mut self, x: usize, y: usize, v: bool) {
        self.data[y * self.width + x] = v;
    }

    pub fn row(&self, y: usize) -> &[bool] {
        &self.data[y * self.width..(y + 1) * self.width]
    }

    pub fn count(&self) -> usize {
        self.data.iter().filter(|&&b| b).count()
    }

    /// Ink bounding box (x, y, w, h), or None when blank.
    pub fn ink_bbox(&self) -> Option<(usize, usize, usize, usize)> {
        let (mut x0, mut y0, mut x1, mut y1) = (usize::MAX, usize::MAX, 0, 0);
        for y in 0..self.height {
            let row = self.row(y);
            if let Some(first) = row.iter().position(|&b| b) {
                let last = row.iter().rposition(|&b| b).unwrap();
                x0 = x0.min(first);
                x1 = x1.max(last);
                y0 = y0.min(y);
                y1 = y;
            }
        }
        (x0 != usize::MAX).then(|| (x0, y0, x1 - x0 + 1, y1 - y0 + 1))
    }

    pub fn crop(&self, x: usize, y: usize, w: usize, h: usize) -> Bitmap {
        let mut data = Vec::with_capacity(w * h);
        for yy in y..y + h {
            data.extend_from_slice(&self.row(yy)[x..x + w]);
        }
        Bitmap { width: w, height: h, data }
    }

    /// Crop to the ink bounding box; returns the offset of the crop.
    pub fn crop_to_ink(&self) -> Option<((usize, usize), Bitmap)> {
        self.ink_bbox().map(|(x, y, w, h)| ((x, y), self.crop(x, y, w, h)))
    }

    /// MSB-first packed row, leftmost pixel first (vendor bit order).
    pub fn packed_row(&self, y: usize) -> Vec<u8> {
        let mut out = vec![0u8; self.width.div_ceil(8)];
        for (x, &b) in self.row(y).iter().enumerate() {
            if b {
                out[x / 8] |= 0x80 >> (x % 8);
            }
        }
        out
    }
}

#[derive(Debug, Clone, PartialEq, Hash)]
pub enum Geometry {
    Raster(Bitmap),
    /// Path segments of one pass, relative to (x, y) of the operation; passes
    /// and laser on/off markers are added when encoding (`protocol::vector_stream`).
    Vector(Vec<Segment>),
}

impl Geometry {
    pub fn hash64(&self) -> u64 {
        let mut h = DefaultHasher::new();
        self.hash(&mut h);
        h.finish()
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct OpPlan {
    pub name: String,
    /// Machine px, top-left of the operation.
    pub x: u32,
    pub y: u32,
    pub width: u32,
    pub height: u32,
    /// Per mille.
    pub power: u16,
    pub depth: u16,
    pub passes: u32,
    pub geometry: Arc<Geometry>,
    /// `geometry.hash64()`, computed once.
    pub geom_hash: u64,
}

impl OpPlan {
    /// Data frames (raster rows / vector batches) of the whole operation.
    ///
    /// Passes repeat inside one job, so that the head never leaves it between
    /// passes: each raster row is sent `passes` times in a row, and the vector
    /// paths are traced `passes` times over.
    pub fn frames(&self) -> Result<Vec<Vec<u8>>, P::ProtocolError> {
        match &*self.geometry {
            Geometry::Raster(bm) => {
                let mut out = Vec::new();
                for y in (0..bm.height).filter(|&y| bm.row(y).iter().any(|&b| b)) {
                    // vendor skips blank rows
                    let row = P::raster_row(y as u16, &bm.packed_row(y), self.power, self.depth)?;
                    out.extend(std::iter::repeat_n(row, self.passes as usize));
                }
                Ok(out)
            }
            Geometry::Vector(segs) => P::vector_stream(segs, self.passes)
                .chunks(P::VECTOR_MAX_SEGMENTS)
                .map(|batch| P::vector_batch(batch, self.power, self.depth))
                .collect(),
        }
    }

    pub fn frame_count(&self) -> usize {
        match &*self.geometry {
            Geometry::Raster(bm) => {
                (0..bm.height).filter(|&y| bm.row(y).iter().any(|&b| b)).count() * self.passes as usize
            }
            Geometry::Vector(segs) => P::vector_stream_len(segs, self.passes).div_ceil(P::VECTOR_MAX_SEGMENTS),
        }
    }

    pub fn bbox(&self) -> Rect {
        Rect { x: self.x, y: self.y, w: self.width, h: self.height }
    }

    /// (burn length, travel between disjoint segments) in mm, for vector ops.
    pub fn lengths_mm(&self) -> (f64, f64) {
        let Geometry::Vector(segs) = &*self.geometry else { return (0.0, 0.0) };
        let (mut burn, mut travel) = (0.0, 0.0);
        let mut last: Option<(u16, u16)> = None;
        for s in segs {
            burn += f64::hypot(s[2] as f64 - s[0] as f64, s[3] as f64 - s[1] as f64);
            if let Some(l) = last.filter(|&l| l != (s[0], s[1])) {
                travel += f64::hypot(s[0] as f64 - l.0 as f64, s[1] as f64 - l.1 as f64);
            }
            last = Some((s[2], s[3]));
        }
        (burn * P::PITCH_MM, travel * P::PITCH_MM)
    }

    pub fn summary(&self) -> String {
        let mm = |v: u32| v as f64 * P::PITCH_MM;
        let detail = match &*self.geometry {
            Geometry::Raster(bm) => format!("raster, {} rows, {} px", self.frame_count(), bm.count()),
            Geometry::Vector(segs) => {
                let (burn, travel) = self.lengths_mm();
                format!("vector, {} segments, burn {burn:.1} mm, travel {travel:.1} mm", segs.len())
            }
        };
        format!(
            "{}: {detail}; at ({:.2}, {:.2}) mm, {:.2}x{:.2} mm; power {} %, depth {}, passes {}",
            self.name,
            mm(self.x),
            mm(self.y),
            mm(self.width),
            mm(self.height),
            self.power as f64 / 10.0,
            self.depth,
            self.passes
        )
    }
}

/// Axis-aligned rectangle in machine px.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Rect {
    pub x: u32,
    pub y: u32,
    pub w: u32,
    pub h: u32,
}

impl Rect {
    pub fn describe_mm(&self) -> String {
        let mm = |v: u32| v as f64 * P::PITCH_MM;
        format!("({:.2}, {:.2}) mm, {:.2} x {:.2} mm", mm(self.x), mm(self.y), mm(self.w), mm(self.h))
    }
}

#[derive(Debug, Clone, Default, PartialEq)]
pub struct Plan {
    pub ops: Vec<OpPlan>,
}

impl Plan {
    pub fn data_frames(&self) -> usize {
        self.ops.iter().map(OpPlan::frame_count).sum()
    }

    pub fn bbox(&self) -> Option<Rect> {
        let x0 = self.ops.iter().map(|o| o.x).min()?;
        let y0 = self.ops.iter().map(|o| o.y).min()?;
        let x1 = self.ops.iter().map(|o| o.x + o.width).max()?;
        let y1 = self.ops.iter().map(|o| o.y + o.height).max()?;
        Some(Rect { x: x0, y: y0, w: x1 - x0, h: y1 - y0 })
    }

    /// Identity of the burn geometry, used by the frame-before-burn interlock.
    pub fn fingerprint(&self) -> u64 {
        let mut h = DefaultHasher::new();
        for op in &self.ops {
            (op.x, op.y, op.width, op.height, op.geom_hash).hash(&mut h);
        }
        h.finish()
    }

    pub fn summary(&self) -> Vec<String> {
        self.ops.iter().map(OpPlan::summary).collect()
    }
}

/// Order polylines nearest-neighbour (reversing when closer) and split them into segments.
pub fn segments_from_polylines(polylines: &[Vec<(u16, u16)>]) -> Vec<Segment> {
    let polys: Vec<&Vec<(u16, u16)>> = polylines.iter().filter(|p| p.len() >= 2).collect();
    let mut used = vec![false; polys.len()];
    let mut out = Vec::new();
    let mut cur = (0.0f64, 0.0f64);
    let d2 = |p: (u16, u16), c: (f64, f64)| (p.0 as f64 - c.0).powi(2) + (p.1 as f64 - c.1).powi(2);
    for _ in 0..polys.len() {
        // nearest start and nearest end; reverse only when the end is strictly closer
        let (mut bs, mut be) = ((f64::INFINITY, 0usize), (f64::INFINITY, 0usize));
        for (i, pl) in polys.iter().enumerate() {
            if used[i] {
                continue;
            }
            let ds = d2(pl[0], cur);
            if ds < bs.0 {
                bs = (ds, i);
            }
            let de = d2(*pl.last().unwrap(), cur);
            if de < be.0 {
                be = (de, i);
            }
        }
        let (i, reversed) = if be.0 < bs.0 { (be.1, true) } else { (bs.1, false) };
        used[i] = true;
        let mut pts: Vec<(u16, u16)> = polys[i].clone();
        if reversed {
            pts.reverse();
        }
        for w in pts.windows(2) {
            if w[0] != w[1] {
                out.push([w[0].0, w[0].1, w[1].0, w[1].1]);
            }
        }
        let last = *pts.last().unwrap();
        cur = (last.0 as f64, last.1 as f64);
    }
    out
}
