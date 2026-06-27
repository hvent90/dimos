// Odin1 frame -> dimos LCM message conversions. Unit decoding (microns, fixed
// point) already happened in the odin1 wrapper, so these are struct mappings.

use std::io::Cursor;

use lcm_msgs::geometry_msgs::{
    Point, Pose, PoseWithCovariance, Quaternion, Twist, TwistWithCovariance, Vector3,
};
use lcm_msgs::nav_msgs::Odometry;
use lcm_msgs::sensor_msgs::{Image, PointCloud2, PointField};
use lcm_msgs::std_msgs::{Header, Time};
use odin1::{DtofFrame, OdometrySample, RgbFrame, SlamCloudFrame};

use crate::Config;

pub struct PublishConfig {
    pub odom_frame_id: String,
    pub child_frame_id: String,
    pub lidar_frame_id: String,
    pub camera_frame_id: String,
    pub confidence_min: u8,
}

impl From<&Config> for PublishConfig {
    fn from(c: &Config) -> Self {
        Self {
            odom_frame_id: c.odom_frame_id.clone(),
            child_frame_id: c.child_frame_id.clone(),
            lidar_frame_id: c.lidar_frame_id.clone(),
            camera_frame_id: c.camera_frame_id.clone(),
            confidence_min: c.confidence_min as u8,
        }
    }
}

fn stamp_from_ns(ns: u64) -> Time {
    Time {
        sec: (ns / 1_000_000_000) as i32,
        nsec: (ns % 1_000_000_000) as i32,
    }
}

fn header(frame_id: &str, stamp: Time) -> Header {
    Header {
        seq: 0,
        stamp,
        frame_id: frame_id.into(),
    }
}

fn field(name: &str, offset: i32) -> PointField {
    PointField {
        name: name.into(),
        offset,
        datatype: PointField::FLOAT32 as u8,
        count: 1,
    }
}

/// Live dtof cloud: x,y,z,intensity, dropping points below the confidence floor.
pub fn dtof_to_pointcloud(frame: &DtofFrame, cfg: &PublishConfig) -> PointCloud2 {
    let n_in = frame.confidence.len();
    let mut data: Vec<u8> = Vec::with_capacity(n_in * 16);
    let mut n: i32 = 0;
    for i in 0..n_in {
        if frame.confidence[i] < cfg.confidence_min {
            continue;
        }
        let x = frame.xyz_m[i * 3];
        let y = frame.xyz_m[i * 3 + 1];
        let z = frame.xyz_m[i * 3 + 2];
        let intensity = frame.intensity[i] as f32;
        data.extend_from_slice(&x.to_le_bytes());
        data.extend_from_slice(&y.to_le_bytes());
        data.extend_from_slice(&z.to_le_bytes());
        data.extend_from_slice(&intensity.to_le_bytes());
        n += 1;
    }
    PointCloud2 {
        header: header(&cfg.lidar_frame_id, stamp_from_ns(frame.stamp_ns)),
        height: 1,
        width: n,
        fields: vec![
            field("x", 0),
            field("y", 4),
            field("z", 8),
            field("intensity", 12),
        ],
        is_bigendian: false,
        point_step: 16,
        row_step: 16 * n,
        data,
        is_dense: false,
    }
}

/// SLAM map cloud: x,y,z,rgb. RGB is packed into a float per the ROS PointCloud2
/// convention (0x00RRGGBB), which rerun and rviz both render as color.
pub fn slam_cloud_to_pointcloud(frame: &SlamCloudFrame, cfg: &PublishConfig) -> PointCloud2 {
    let n = frame.points.len() as i32;
    let mut data: Vec<u8> = Vec::with_capacity(frame.points.len() * 16);
    for p in &frame.points {
        data.extend_from_slice(&p.xyz_m[0].to_le_bytes());
        data.extend_from_slice(&p.xyz_m[1].to_le_bytes());
        data.extend_from_slice(&p.xyz_m[2].to_le_bytes());
        let [r, g, b, _a] = p.rgba;
        let rgb = ((r as u32) << 16) | ((g as u32) << 8) | (b as u32);
        data.extend_from_slice(&f32::from_bits(rgb).to_le_bytes());
    }
    PointCloud2 {
        header: header(&cfg.odom_frame_id, stamp_from_ns(frame.stamp_ns)),
        height: 1,
        width: n,
        fields: vec![
            field("x", 0),
            field("y", 4),
            field("z", 8),
            field("rgb", 12),
        ],
        is_bigendian: false,
        point_step: 16,
        row_step: 16 * n,
        data,
        is_dense: false,
    }
}

/// The Odin's RGB stream is JPEG-compressed on the wire (the upstream `nv12`
/// field name notwithstanding). Decode to rgb8/mono8 so consumers and rerun get
/// a usable raster. Falls back to publishing the raw payload if it is not JPEG.
pub fn rgb_to_image(frame: &RgbFrame, cfg: &PublishConfig) -> Image {
    let payload = &frame.nv12;
    let is_jpeg = payload.len() >= 2 && payload[0] == 0xFF && payload[1] == 0xD8;
    if is_jpeg {
        if let Some(img) = decode_jpeg(payload, &cfg.camera_frame_id, stamp_from_ns(frame.stamp_ns))
        {
            return img;
        }
    }
    Image {
        header: header(&cfg.camera_frame_id, stamp_from_ns(frame.stamp_ns)),
        height: frame.height as i32,
        width: frame.width as i32,
        encoding: if is_jpeg { "jpeg" } else { "nv12" }.into(),
        is_bigendian: 0,
        step: frame.width as i32,
        data: payload.clone(),
    }
}

fn decode_jpeg(buf: &[u8], frame_id: &str, stamp: Time) -> Option<Image> {
    use jpeg_decoder::PixelFormat;
    let mut decoder = jpeg_decoder::Decoder::new(Cursor::new(buf));
    let pixels = decoder.decode().ok()?;
    let info = decoder.info()?;
    let (encoding, step) = match info.pixel_format {
        PixelFormat::RGB24 => ("rgb8", info.width as i32 * 3),
        PixelFormat::L8 => ("mono8", info.width as i32),
        _ => return None,
    };
    Some(Image {
        header: header(frame_id, stamp),
        height: info.height as i32,
        width: info.width as i32,
        encoding: encoding.into(),
        is_bigendian: 0,
        step,
        data: pixels,
    })
}

pub fn odom_to_odometry(s: &OdometrySample, cfg: &PublishConfig) -> Odometry {
    Odometry {
        header: header(&cfg.odom_frame_id, stamp_from_ns(s.stamp_ns)),
        child_frame_id: cfg.child_frame_id.clone(),
        pose: PoseWithCovariance {
            pose: Pose {
                position: Point {
                    x: s.position[0],
                    y: s.position[1],
                    z: s.position[2],
                },
                orientation: Quaternion {
                    x: s.orientation[0],
                    y: s.orientation[1],
                    z: s.orientation[2],
                    w: s.orientation[3],
                },
            },
            covariance: s.pose_cov,
        },
        twist: TwistWithCovariance {
            twist: Twist {
                linear: Vector3 {
                    x: s.linear_velocity[0],
                    y: s.linear_velocity[1],
                    z: s.linear_velocity[2],
                },
                angular: Vector3 {
                    x: s.angular_velocity[0],
                    y: s.angular_velocity[1],
                    z: s.angular_velocity[2],
                },
            },
            covariance: s.twist_cov,
        },
    }
}
