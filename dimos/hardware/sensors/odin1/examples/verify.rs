use odin1::{ConnectOptions, DepthOdr, Device, Frame, Mode, Streams};
use std::collections::HashMap;
use std::time::{Duration, Instant};

fn main() {
    let opts = ConnectOptions {
        mode: Mode::Slam,
        streams: Streams {
            dtof: true,
            rgb: true,
            imu: true,
            odometry: true,
            odometry_highfreq: false,
            slam_cloud: true,
        },
        calib_out_path: "/tmp".into(),
        discovery_timeout_s: 10.0,
        dtof_odr: DepthOdr::Hz10,
        channel_capacity: 256,
    };
    eprintln!("[verify] connecting (move the sensor a little so SLAM initializes)...");
    let (_dev, rx) = match Device::connect(&opts) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("[verify] CONNECT FAILED: {e}");
            std::process::exit(1);
        }
    };
    eprintln!("[verify] connected. collecting 10s...");

    let dur = Duration::from_secs(10);
    let start = Instant::now();
    let mut counts: HashMap<&str, u64> = HashMap::new();
    let (mut sd, mut ss, mut sr, mut si, mut so) = (None, None, None, None, None);
    while start.elapsed() < dur {
        match rx.recv_timeout(Duration::from_millis(500)) {
            Ok(Frame::Dtof(d)) => {
                *counts.entry("dtof").or_default() += 1;
                sd = Some(d);
            }
            Ok(Frame::SlamCloud(c)) => {
                *counts.entry("slam").or_default() += 1;
                ss.get_or_insert(c);
            }
            Ok(Frame::Rgb(r)) => {
                *counts.entry("rgb").or_default() += 1;
                sr.get_or_insert(r);
            }
            Ok(Frame::Imu(i)) => {
                *counts.entry("imu").or_default() += 1;
                si.get_or_insert(i);
            }
            Ok(Frame::Odometry(o)) => {
                *counts.entry("odom").or_default() += 1;
                so.get_or_insert(o);
            }
            Err(_) => {}
        }
    }
    let secs = start.elapsed().as_secs_f64();
    eprintln!("\n=== per-stream rates over {secs:.1}s ===");
    for k in ["dtof", "slam", "rgb", "imu", "odom"] {
        let c = *counts.get(k).unwrap_or(&0);
        eprintln!("  {k:5} {c:6} frames   {:6.1} Hz", c as f64 / secs);
    }
    eprintln!("\n=== sanity checks ===");
    match &sd {
        Some(d) => {
            let n = d.confidence.len();
            let dnz = d
                .depth_m
                .iter()
                .filter(|v| **v != 0.0 && v.is_finite())
                .count();
            let xnz = d
                .xyz_m
                .iter()
                .filter(|v| **v != 0.0 && v.is_finite())
                .count();
            let cnz = d.confidence.iter().filter(|v| **v > 0).count();
            let inz = d.intensity.iter().filter(|v| **v > 0).count();
            let dmax = d
                .depth_m
                .iter()
                .cloned()
                .filter(|v| v.is_finite())
                .fold(0f32, f32::max);
            eprintln!("  dtof: {n} pts | nonzero: depth={dnz} xyz={xnz} conf={cnz} intensity={inz} | depth_max={dmax:.2}m");
        }
        None => eprintln!("  dtof: NO FRAMES"),
    }
    match &ss {
        Some(c) => eprintln!(
            "  slam: {} pts | first={:?}",
            c.points.len(),
            c.points.first()
        ),
        None => eprintln!("  slam: NO FRAMES"),
    }
    match &sr {
        Some(r) => {
            let jpeg = r.nv12.len() >= 2 && r.nv12[0] == 0xFF && r.nv12[1] == 0xD8;
            eprintln!(
                "  rgb:  {}x{} | {} bytes | JPEG magic={}",
                r.width,
                r.height,
                r.nv12.len(),
                jpeg
            );
        }
        None => eprintln!("  rgb:  NO FRAMES"),
    }
    match &si {
        Some(i) => {
            let m = (i.accel[0].powi(2) + i.accel[1].powi(2) + i.accel[2].powi(2)).sqrt();
            eprintln!("  imu:  |a|={m:.2} m/s^2 (expect ~9.8)");
        }
        None => eprintln!("  imu:  NO FRAMES"),
    }
    match &so {
        Some(o) => {
            let q = (o.orientation[0].powi(2)
                + o.orientation[1].powi(2)
                + o.orientation[2].powi(2)
                + o.orientation[3].powi(2))
            .sqrt();
            eprintln!("  odom: pos {:?} | |q|={q:.4} (expect ~1.0)", o.position);
        }
        None => eprintln!("  odom: NO FRAMES"),
    }
    eprintln!("\n[verify] done.");
    std::process::exit(0);
}
