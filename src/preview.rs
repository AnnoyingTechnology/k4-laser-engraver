//! Reconstruct what the machine will burn from the exact frames sent to it.

use std::path::Path;

use anyhow::Result;

use crate::plan::{Bitmap, Rect};
use crate::protocol::{self as P, Fields, Frame};

#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct Stats {
    pub jobs: usize,
    pub rows: usize,
    pub batches: usize,
    pub segments: usize,
    pub out_of_area: usize,
}

/// Replay START / RASTER_ROW / VECTOR frames into a burn mask.
pub fn render_frames(frames: &[Frame], area_px: (u32, u32)) -> (Bitmap, Stats) {
    let (w, h) = (area_px.0 as i64, area_px.1 as i64);
    let mut burn = Bitmap::new(w as usize, h as usize);
    let mut st = Stats::default();
    let mut origin = (0i64, 0i64);
    for fr in frames {
        match fr.fields() {
            Fields::Start { x, y } => {
                origin = (x as i64, y as i64);
                st.jobs += 1;
            }
            Fields::Raster { row, bits, .. } => {
                st.rows += 1;
                let y = origin.1 + row as i64;
                let mut outside = !(0..h).contains(&y);
                for (i, byte) in bits.iter().enumerate() {
                    for b in 0..8 {
                        if byte & (0x80 >> b) == 0 {
                            continue;
                        }
                        let x = origin.0 + (i * 8 + b) as i64;
                        if x >= w || outside {
                            outside = true;
                            continue;
                        }
                        burn.set(x as usize, y as usize, true);
                    }
                }
                st.out_of_area += outside as usize;
            }
            Fields::Vector { segments, .. } => {
                st.batches += 1;
                st.segments += segments.len();
                for s in segments {
                    let (x0, y0) = (s[0] as i64 + origin.0, s[1] as i64 + origin.1);
                    let (x1, y1) = (s[2] as i64 + origin.0, s[3] as i64 + origin.1);
                    let n = (x1 - x0).abs().max((y1 - y0).abs());
                    let mut outside = false;
                    for k in 0..=n {
                        let t = if n == 0 { 0.0 } else { k as f64 / n as f64 };
                        let x = (x0 as f64 + (x1 - x0) as f64 * t).round_ties_even() as i64;
                        let y = (y0 as f64 + (y1 - y0) as f64 * t).round_ties_even() as i64;
                        if (0..w).contains(&x) && (0..h).contains(&y) {
                            burn.set(x as usize, y as usize, true);
                        } else {
                            outside = true;
                        }
                    }
                    st.out_of_area += outside as usize;
                }
            }
            _ => {}
        }
    }
    (burn, st)
}

/// White background, black burn, 10 mm grid, light-blue frame rectangle; 508 dpi.
pub fn save_png(burn: &Bitmap, path: &Path, frame: Option<Rect>) -> Result<()> {
    let (w, h) = (burn.width as u32, burn.height as u32);
    let step = (10.0 / P::PITCH_MM) as u32;
    let mut img = image::RgbImage::from_fn(w, h, |x, y| {
        if x % step == 0 || y % step == 0 { image::Rgb([235, 235, 235]) } else { image::Rgb([255, 255, 255]) }
    });
    if let Some(r) = frame {
        let blue = image::Rgb([80, 160, 255]);
        let (x1, y1) = ((r.x + r.w).min(w - 1), (r.y + r.h).min(h - 1));
        for x in r.x..=x1 {
            img.put_pixel(x, r.y.min(h - 1), blue);
            img.put_pixel(x, y1, blue);
        }
        for y in r.y..=y1 {
            img.put_pixel(r.x.min(w - 1), y, blue);
            img.put_pixel(x1, y, blue);
        }
    }
    for y in 0..burn.height {
        for x in 0..burn.width {
            if burn.get(x, y) {
                img.put_pixel(x as u32, y as u32, image::Rgb([0, 0, 0]));
            }
        }
    }
    // 508 dpi = 20000 px/m, recorded in pHYs so viewers print at true scale
    let file = std::io::BufWriter::new(std::fs::File::create(path)?);
    let mut enc = png::Encoder::new(file, w, h);
    enc.set_color(png::ColorType::Rgb);
    enc.set_depth(png::BitDepth::Eight);
    enc.set_pixel_dims(Some(png::PixelDimensions { xppu: 20_000, yppu: 20_000, unit: png::Unit::Meter }));
    enc.write_header()?.write_image_data(img.as_raw())?;
    Ok(())
}
