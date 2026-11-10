//! Job duration estimates, learned from real jobs.
//!
//! The machine reports no timing, and the unit of `depth` is unknown, so the
//! durations come from measurements. Every data frame sent to a real machine
//! is timed from send to ACK, and a linear model with non-negative
//! coefficients is fitted per mode on the features of [`Sample`]. The setup of
//! each job (move, START) is averaged. The model is kept in
//! `$XDG_STATE_HOME/k4laser/timing.toml`; delete that file to start over.

use std::cell::RefCell;
use std::collections::HashMap;
use std::path::PathBuf;
use std::time::Duration;

use anyhow::{Result, anyhow};
use serde::{Deserialize, Serialize};

use crate::plan::{OpPlan, Plan};
use crate::protocol::{self as P, Fields, Frame};

const N: usize = 4;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sample {
    pub vector: bool,
    /// 1 (per frame), px moved with the laser off, burnt px, burnt px × depth.
    pub x: [f64; N],
}

/// Features of each data frame of one pass, in order.
pub fn samples(frames: &[Vec<u8>]) -> Vec<Sample> {
    let mut laser = true; // as after START, see `preview::render_frames`
    let mut head = (0.0, 0.0);
    frames
        .iter()
        .map(|bytes| {
            let fr = Frame { op: bytes[0], data: bytes.clone() };
            match fr.fields() {
                Fields::Raster { depth, bits, .. } => {
                    let set: Vec<usize> =
                        (0..bits.len() * 8).filter(|&i| bits[i / 8] & (0x80 >> (i % 8)) != 0).collect();
                    let burn = set.len() as f64;
                    let span = set.last().map_or(0.0, |&l| (l - set[0] + 1) as f64);
                    Sample { vector: false, x: [1.0, span - burn, burn, burn * depth as f64] }
                }
                Fields::Vector { depth, segments, .. } => {
                    let (mut off, mut on) = (0.0, 0.0);
                    for s in segments {
                        match s {
                            P::VECTOR_LASER_ON => laser = true,
                            P::VECTOR_LASER_OFF => laser = false,
                            _ => {
                                let (a, b) = ((s[0] as f64, s[1] as f64), (s[2] as f64, s[3] as f64));
                                let d = f64::hypot(a.0 - head.0, a.1 - head.1) + f64::hypot(b.0 - a.0, b.1 - a.1);
                                *(if laser { &mut on } else { &mut off }) += d;
                                head = b;
                            }
                        }
                    }
                    Sample { vector: true, x: [1.0, off, on, on * depth as f64] }
                }
                _ => Sample { vector: false, x: [1.0, 0.0, 0.0, 0.0] },
            }
        })
        .collect()
}

/// Normal equations of a least-squares fit.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
struct Fit {
    samples: u64,
    xtx: [[f64; N]; N],
    xty: [f64; N],
}

impl Fit {
    fn add(&mut self, x: &[f64; N], secs: f64) {
        self.samples += 1;
        for i in 0..N {
            self.xty[i] += x[i] * secs;
            for j in 0..N {
                self.xtx[i][j] += x[i] * x[j];
            }
        }
    }

    /// Coefficients, all ≥ 0; a negative one is dropped and the rest refitted.
    fn coef(&self) -> Option<[f64; N]> {
        // one frame already gives a rough fit; it sharpens with every job
        if self.samples == 0 {
            return None;
        }
        let mut free = [true; N];
        loop {
            let c = self.solve(&free)?;
            match (0..N).filter(|&i| c[i] < 0.0).min_by(|&a, &b| c[a].total_cmp(&c[b])) {
                Some(i) => free[i] = false,
                None => return Some(c),
            }
        }
    }

    /// Least squares over the `free` features (Jacobi-scaled, light ridge).
    fn solve(&self, free: &[bool; N]) -> Option<[f64; N]> {
        let idx: Vec<usize> = (0..N).filter(|&i| free[i]).collect();
        let n = idx.len();
        let scale: Vec<f64> = idx.iter().map(|&i| self.xtx[i][i].sqrt().max(1e-12)).collect();
        let mut m: Vec<Vec<f64>> = (0..n)
            .map(|r| {
                let mut row: Vec<f64> = (0..n).map(|c| self.xtx[idx[r]][idx[c]] / (scale[r] * scale[c])).collect();
                row[r] += 1e-9;
                row.push(self.xty[idx[r]] / scale[r]);
                row
            })
            .collect();
        for col in 0..n {
            let p = (col..n).max_by(|&a, &b| m[a][col].abs().total_cmp(&m[b][col].abs()))?;
            m.swap(col, p);
            let pivot = m[col].clone();
            for (r, row) in m.iter_mut().enumerate() {
                if r != col {
                    let f = row[col] / pivot[col];
                    row.iter_mut().zip(&pivot).skip(col).for_each(|(v, p)| *v -= f * p);
                }
            }
        }
        let mut out = [0.0; N];
        for (k, &i) in idx.iter().enumerate() {
            out[i] = m[k][n] / m[k][k] / scale[k];
        }
        out.iter().all(|v| v.is_finite()).then_some(out)
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
#[serde(default)]
pub struct Timing {
    raster: Fit,
    vector: Fit,
    jobs: u64,
    job_secs: f64,
    #[serde(skip)]
    cache: RefCell<PassCache>,
}

/// Data time of an operation by (geometry hash, depth, passes), or the mode
/// without a fit.
type PassCache = HashMap<(u64, u16, u32), Result<f64, &'static str>>;

/// Estimated duration of a plan, or what it still needs.
#[derive(Debug, Clone, PartialEq)]
pub enum Estimate {
    Known(Duration),
    /// No timed job of this kind yet ("raster" or "vector").
    Unknown(&'static str),
}

impl Timing {
    pub fn record(&mut self, s: &Sample, secs: f64) {
        if s.vector { &mut self.vector } else { &mut self.raster }.add(&s.x, secs);
        self.cache.get_mut().clear();
    }

    pub fn record_setup(&mut self, secs: f64) {
        self.jobs += 1;
        self.job_secs += secs;
    }

    /// Frames timed so far.
    pub fn frames(&self) -> u64 {
        self.raster.samples + self.vector.samples
    }

    pub fn estimate(&self, plan: &Plan) -> Estimate {
        self.estimate_ops(&plan.ops)
    }

    pub fn estimate_ops(&self, ops: &[OpPlan]) -> Estimate {
        let setup = self.job_secs / self.jobs.max(1) as f64;
        let mut secs = 0.0;
        for op in ops {
            let key = (op.geom_hash, op.depth, op.passes);
            let data = *self.cache.borrow_mut().entry(key).or_insert_with(|| self.data_secs(op));
            match data {
                Ok(s) => secs += setup + s,
                Err(kind) => return Estimate::Unknown(kind),
            }
        }
        Estimate::Known(Duration::from_secs_f64(secs))
    }

    /// Data frames of the operation, or the mode that has no fit yet.
    fn data_secs(&self, op: &OpPlan) -> Result<f64, &'static str> {
        let (raster, vector) = (self.raster.coef(), self.vector.coef());
        let Ok(frames) = op.frames() else { return Ok(0.0) }; // compiled plans encode
        let mut secs = 0.0;
        for s in samples(&frames) {
            let c = if s.vector { vector.ok_or("vector")? } else { raster.ok_or("raster")? };
            secs += (0..N).map(|i| c[i] * s.x[i]).sum::<f64>();
        }
        Ok(secs)
    }
}

pub fn path() -> PathBuf {
    let base = std::env::var_os("XDG_STATE_HOME")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".local/state")))
        .unwrap_or_else(|| PathBuf::from("."));
    base.join("k4laser").join("timing.toml")
}

/// The saved model; empty when there is none or it cannot be read.
pub fn load() -> Timing {
    std::fs::read_to_string(path()).ok().and_then(|t| toml::from_str(&t).ok()).unwrap_or_default()
}

pub fn save(t: &Timing) -> Result<()> {
    let p = path();
    if let Some(dir) = p.parent() {
        std::fs::create_dir_all(dir)?;
    }
    let tmp = p.with_extension("toml.tmp");
    std::fs::write(&tmp, toml::to_string(t)?).map_err(|e| anyhow!("{}: {e}", tmp.display()))?;
    std::fs::rename(&tmp, &p)?;
    Ok(())
}

/// "≈ 4 min 10 s"; whole seconds, minutes from one minute, hours from one hour.
pub fn human(d: Duration) -> String {
    let s = d.as_secs_f64().round() as u64;
    match s {
        0..60 => format!("≈ {s} s"),
        60..3600 => format!("≈ {} min {:02} s", s / 60, s % 60),
        _ => format!("≈ {} h {:02} min", s / 3600, s % 3600 / 60),
    }
}

impl std::fmt::Display for Estimate {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Estimate::Known(d) => f.write_str(&human(*d)),
            Estimate::Unknown(kind) => write!(f, "unknown until a {kind} job has run"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fit_recovers_a_linear_model() {
        // 20 ms per frame, 0 off-travel, 1 ms per burnt px, 0.1 ms per px × depth
        let truth = [0.02, 0.0, 0.001, 0.0001];
        let mut fit = Fit::default();
        for k in 0..40u32 {
            let (burn, depth) = ((k * 37 % 200) as f64, (5 + k % 4 * 20) as f64);
            let x = [1.0, (k % 7) as f64 * 10.0, burn, burn * depth];
            fit.add(&x, (0..N).map(|i| truth[i] * x[i]).sum());
        }
        let c = fit.coef().unwrap();
        for i in 0..N {
            assert!((c[i] - truth[i]).abs() < 1e-6, "{c:?}");
        }
    }

    #[test]
    fn coefficients_stay_non_negative() {
        let mut fit = Fit::default();
        // time falls with travel: the travel term is dropped, not made negative
        for k in 0..20 {
            let x = [1.0, k as f64, 10.0, 100.0];
            fit.add(&x, 1.0 - 0.01 * k as f64);
        }
        assert!(fit.coef().unwrap().iter().all(|&v| v >= 0.0));
        assert_eq!(Fit::default().coef(), None);
    }

    #[test]
    fn samples_follow_the_stream() {
        let bits = [0b0011_0000u8, 0b0000_1000];
        let row = P::raster_row(0, &bits, 500, 10).unwrap();
        let paths = [[0, 0, 10, 0], [20, 0, 30, 0]];
        let mut frames = vec![row];
        frames.extend(P::vector_stream(&paths, 1).chunks(5).map(|b| P::vector_batch(b, 500, 7).unwrap()));
        let s = samples(&frames);
        // raster: px 2, 3, 12 set; span 11, 3 burnt
        assert_eq!(s[0], Sample { vector: false, x: [1.0, 8.0, 3.0, 30.0] });
        // vector: 20 px burnt; 10 px between the paths and 30 px back home with the laser off
        let (off, on): (f64, f64) = s[1..].iter().fold((0.0, 0.0), |(a, b), s| (a + s.x[1], b + s.x[2]));
        assert_eq!((off, on), (40.0, 20.0));
        assert!(s[1..].iter().all(|s| s.vector && s.x[3] == s.x[2] * 7.0));
    }

    #[test]
    fn human_durations() {
        assert_eq!(human(Duration::from_secs_f64(9.6)), "≈ 10 s");
        assert_eq!(human(Duration::from_secs(250)), "≈ 4 min 10 s");
        assert_eq!(human(Duration::from_secs(3 * 3600 + 120)), "≈ 3 h 02 min");
    }
}
