//! Artwork transforms applied after rendering: perimeters (offset outlines),
//! mirror and quarter-turn rotation.

use i_overlay::core::fill_rule::FillRule;
use i_overlay::float::simplify::SimplifyShape;
use i_overlay::mesh::float::outline::offset::OutlineOffset;
use i_overlay::mesh::float::stroke::offset::StrokeOffset;
use i_overlay::mesh::float::style::{LineCap, LineJoin, OutlineStyle, StrokeStyle};
use serde::{Deserialize, Serialize};

use crate::plan::Bitmap;
use crate::sources::{Artwork, Polyline};

/// Where extra perimeters go, relative to the material enclosed by closed paths.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Side {
    /// Away from the material: a cut-out part keeps its exact size.
    #[default]
    Outside,
    /// Into the material: a hole keeps its exact size.
    Inside,
    /// Both ways, centred on the path.
    Centered,
}

impl Side {
    pub fn label(self) -> &'static str {
        match self {
            Side::Outside => "Outside",
            Side::Inside => "Inside",
            Side::Centered => "Centered",
        }
    }
}

/// Offsets (mm, positive = outward) of `count` perimeters `spacing` apart.
pub fn perimeter_offsets(count: u32, spacing: f64, side: Side) -> Vec<f64> {
    let n = count.max(1);
    (0..n)
        .map(|k| match side {
            Side::Outside => k as f64 * spacing,
            Side::Inside => -(k as f64) * spacing,
            Side::Centered => (k as f64 - (n - 1) as f64 / 2.0) * spacing,
        })
        .collect()
}

fn is_closed(pl: &Polyline) -> bool {
    pl.len() >= 4 && {
        let (a, b) = (pl[0], pl[pl.len() - 1]);
        (a.0 - b.0).abs() < 1e-9 && (a.1 - b.1).abs() < 1e-9
    }
}

fn close(contour: Vec<[f64; 2]>) -> Polyline {
    let mut pl: Polyline = contour.into_iter().map(|[x, y]| (x, y)).collect();
    if let Some(&first) = pl.first() {
        pl.push(first);
    }
    pl
}

/// Concentric copies of vector paths, like the walls of a 3D print.
///
/// Closed paths are taken together as a region (even-odd), so "outside" grows
/// an outer boundary and shrinks a hole. Open paths have no inside: each extra
/// perimeter is a loop around the path at that distance.
pub fn perimeters(polylines: Vec<Polyline>, count: u32, spacing: f64, side: Side) -> Vec<Polyline> {
    if count <= 1 || spacing <= 0.0 {
        return polylines;
    }
    let offsets = perimeter_offsets(count, spacing, side);
    let (closed, open): (Vec<Polyline>, Vec<Polyline>) = polylines.into_iter().partition(is_closed);
    let mut out = Vec::new();
    // a join segment of at most ~0.02 mm at the smallest radius
    let round = LineJoin::Round(0.02 / spacing.max(0.02));
    if !closed.is_empty() {
        let rings: Vec<Vec<[f64; 2]>> =
            closed.iter().map(|pl| pl[..pl.len() - 1].iter().map(|&(x, y)| [x, y]).collect()).collect();
        let region = rings.simplify_shape(FillRule::EvenOdd);
        for &o in &offsets {
            if o == 0.0 {
                out.extend(closed.iter().cloned());
                continue;
            }
            let style = OutlineStyle::new(o).line_join(round.clone());
            for shape in region.outline(&style) {
                out.extend(shape.into_iter().map(close));
            }
        }
    }
    if !open.is_empty() {
        let mut dists: Vec<f64> = offsets.iter().map(|o| o.abs()).collect();
        dists.sort_by(f64::total_cmp);
        dists.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
        for d in dists {
            if d == 0.0 {
                out.extend(open.iter().cloned());
                continue;
            }
            let style = StrokeStyle::new(2.0 * d)
                .line_join(round.clone())
                .start_cap(LineCap::Round(0.1))
                .end_cap(LineCap::Round(0.1));
            for pl in &open {
                let path: Vec<[f64; 2]> = pl.iter().map(|&(x, y)| [x, y]).collect();
                for shape in path.stroke(style.clone(), false) {
                    out.extend(shape.into_iter().map(close));
                }
            }
        }
    }
    out
}

// --------------------------------------------------------- mirror / rotate

/// Left-right mirror.
pub fn mirror_bitmap(bm: &Bitmap) -> Bitmap {
    Bitmap::from_fn(bm.width, bm.height, |x, y| bm.get(bm.width - 1 - x, y))
}

/// Quarter turn clockwise (as seen on the bed, Y down).
pub fn rotate_bitmap_cw(bm: &Bitmap) -> Bitmap {
    Bitmap::from_fn(bm.height, bm.width, |x, y| bm.get(y, bm.height - 1 - x))
}

fn normalize(polys: Vec<Polyline>) -> Vec<Polyline> {
    let x0 = polys.iter().flatten().map(|p| p.0).fold(f64::INFINITY, f64::min);
    let y0 = polys.iter().flatten().map(|p| p.1).fold(f64::INFINITY, f64::min);
    if !x0.is_finite() {
        return polys;
    }
    polys.into_iter().map(|pl| pl.into_iter().map(|(x, y)| (x - x0, y - y0)).collect()).collect()
}

/// Mirror (left-right) then rotate clockwise by `degrees` (a multiple of 90).
pub fn transform(art: Artwork, mirror: bool, degrees: u16) -> Artwork {
    let turns = (degrees / 90) % 4;
    if !mirror && turns == 0 {
        return art;
    }
    let bitmap = |mut bm: Bitmap| {
        if mirror {
            bm = mirror_bitmap(&bm);
        }
        for _ in 0..turns {
            bm = rotate_bitmap_cw(&bm);
        }
        bm
    };
    match art {
        Artwork::Raster(bm) => Artwork::Raster(bitmap(bm)),
        Artwork::Levels(levels) => Artwork::Levels(levels.into_iter().map(bitmap).collect()),
        Artwork::Vector(polys) => {
            let f = |(mut x, mut y): (f64, f64)| {
                if mirror {
                    x = -x;
                }
                for _ in 0..turns {
                    (x, y) = (-y, x);
                }
                (x, y)
            };
            Artwork::Vector(normalize(polys.into_iter().map(|pl| pl.into_iter().map(f).collect()).collect()))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square(x: f64, y: f64, s: f64) -> Polyline {
        vec![(x, y), (x + s, y), (x + s, y + s), (x, y + s), (x, y)]
    }

    fn bbox(polys: &[Polyline]) -> (f64, f64, f64, f64) {
        let pts = polys.iter().flatten();
        let f = |a: fn(&(f64, f64)) -> f64, min: bool| {
            pts.clone()
                .map(a)
                .fold(if min { f64::INFINITY } else { f64::NEG_INFINITY }, if min { f64::min } else { f64::max })
        };
        (f(|p| p.0, true), f(|p| p.1, true), f(|p| p.0, false), f(|p| p.1, false))
    }

    #[test]
    fn outside_grows_parts_and_shrinks_holes() {
        // 10 mm square with a 4 mm hole
        let polys = vec![square(0.0, 0.0, 10.0), square(3.0, 3.0, 4.0)];
        let out = perimeters(polys, 2, 0.5, Side::Outside);
        // original two rings + offset boundary + offset hole
        assert_eq!(out.len(), 4);
        let (x0, y0, x1, y1) = bbox(&out);
        assert!(
            (x0 + 0.5).abs() < 1e-3 && (y0 + 0.5).abs() < 1e-3 && (x1 - 10.5).abs() < 1e-3 && (y1 - 10.5).abs() < 1e-3
        );
        // the offset hole ring lies inside the original hole
        let hole = out[2..].iter().find(|pl| bbox(std::slice::from_ref(pl)).2 < 8.0).expect("offset hole");
        let (hx0, _, hx1, _) = bbox(std::slice::from_ref(hole));
        assert!((hx0 - 3.5).abs() < 1e-3 && (hx1 - 6.5).abs() < 1e-3, "{hx0} {hx1}");
        assert!(out.iter().all(is_closed));
    }

    #[test]
    fn inside_and_centered_offsets() {
        assert_eq!(perimeter_offsets(3, 0.1, Side::Inside), [0.0, -0.1, -0.2]);
        let c = perimeter_offsets(3, 0.1, Side::Centered);
        assert!((c[0] + 0.1).abs() < 1e-12 && c[1].abs() < 1e-12 && (c[2] - 0.1).abs() < 1e-12);
        let out = perimeters(vec![square(0.0, 0.0, 10.0)], 3, 1.0, Side::Inside);
        let (x0, _, x1, _) = bbox(&out[2..]);
        assert!((x0 - 2.0).abs() < 1e-3 && (x1 - 8.0).abs() < 1e-3);
    }

    #[test]
    fn open_paths_get_loops() {
        let out = perimeters(vec![vec![(0.0, 0.0), (10.0, 0.0)]], 2, 0.5, Side::Outside);
        assert_eq!(out.len(), 2);
        assert!(is_closed(&out[1]));
        let (x0, y0, x1, y1) = bbox(&out[1..]);
        assert!((y0 + 0.5).abs() < 1e-2 && (y1 - 0.5).abs() < 1e-2 && x0 < -0.4 && x1 > 10.4);
    }

    #[test]
    fn rotation_and_mirror() {
        let bm = Bitmap::from_fn(3, 2, |x, y| x == 2 && y == 0); // top-right
        let r = rotate_bitmap_cw(&bm);
        assert_eq!((r.width, r.height), (2, 3));
        assert!(r.get(1, 2)); // bottom-right
        assert!(mirror_bitmap(&bm).get(0, 0));
        let Artwork::Vector(v) = transform(Artwork::Vector(vec![vec![(0.0, 0.0), (4.0, 0.0)]]), false, 90) else {
            panic!()
        };
        assert_eq!(v[0], [(0.0, 0.0), (0.0, 4.0)]);
    }
}
