// dimos native module for the Manifold Tech Odin1.
//
// The Odin1 runs SLAM onboard, so this is a thin source: connect via the odin1
// wrapper, pump typed frames off the device, convert to dimos messages, and
// publish the live depth cloud, the SLAM map cloud, the camera image, and
// onboard odometry. No host-side LIO.

mod convert;

use dimos_module::{native_config, run, LcmTransport, Module};
use lcm_msgs::nav_msgs::Odometry;
use lcm_msgs::sensor_msgs::{Image, PointCloud2};
use odin1::{ConnectOptions, DepthOdr, Frame, Mode, Streams};

// native_config: every field required and supplied by the Python wrapper over
// stdin. No Rust-side defaults, no Option.
#[native_config]
struct Config {
    /// "slam" exposes raw streams plus onboard odometry. "raw" omits odometry.
    mode: String,
    /// Fixed odometry frame (also the SLAM map cloud frame). Named to avoid the
    /// reserved dimos base-config `frame_id`, which is CLI-only and stripped from
    /// the stdin JSON this module reads.
    odom_frame_id: String,
    /// Moving body frame the odometry pose targets.
    child_frame_id: String,
    /// Frame the dtof point cloud is stamped in.
    lidar_frame_id: String,
    /// Frame the camera image is stamped in.
    camera_frame_id: String,
    /// Drop dtof points below this confidence. SDK suggests ~30-35.
    #[validate(range(min = 0, max = 255))]
    confidence_min: u32,
    /// Publish the RGB camera image.
    publish_image: bool,
    /// Publish odometry at IMU rate instead of the ~10Hz SLAM rate.
    odometry_highfreq: bool,
    /// Seconds to wait for a device on the USB bus before failing.
    #[validate(range(min = 0.0, max = 120.0))]
    discovery_timeout_s: f64,
    /// Where the per-device calib.yaml retrieved on connect is written.
    calib_out_path: String,
    /// DTOF depth output rate in Hz: "10", "14.5", or "29". Required for the
    /// depth sensor to produce frames.
    depth_rate_hz: String,
    /// Bounded frame-channel capacity. Newest frames drop when the consumer lags.
    #[validate(range(min = 1, max = 65536))]
    channel_capacity: u32,
}

impl Config {
    fn depth_odr(&self) -> DepthOdr {
        match self.depth_rate_hz.as_str() {
            "29" => DepthOdr::Hz29,
            "14.5" => DepthOdr::Hz14_5,
            _ => DepthOdr::Hz10,
        }
    }

    fn to_connect_options(&self) -> ConnectOptions {
        let mode = if self.mode == "raw" {
            Mode::Raw
        } else {
            Mode::Slam
        };
        let slam = mode == Mode::Slam;
        ConnectOptions {
            mode,
            streams: Streams {
                dtof: true,
                rgb: self.publish_image,
                imu: false,
                odometry: slam && !self.odometry_highfreq,
                odometry_highfreq: slam && self.odometry_highfreq,
                // The SLAM synchronizer needs a cloud stream to emit odometry.
                slam_cloud: slam,
            },
            calib_out_path: self.calib_out_path.clone(),
            discovery_timeout_s: self.discovery_timeout_s,
            dtof_odr: self.depth_odr(),
            channel_capacity: self.channel_capacity as usize,
        }
    }
}

#[derive(Module)]
#[module(setup = start)]
struct OdinModule {
    // Live per-frame depth cloud (sensor frame), x/y/z/intensity.
    #[output(encode = PointCloud2::encode)]
    lidar: dimos_module::Output<PointCloud2>,

    // Onboard SLAM map cloud (map frame), x/y/z/rgb.
    #[output(encode = PointCloud2::encode)]
    slam_cloud: dimos_module::Output<PointCloud2>,

    #[output(encode = Image::encode)]
    color_image: dimos_module::Output<Image>,

    #[output(encode = Odometry::encode)]
    odometry: dimos_module::Output<Odometry>,

    #[config]
    config: Config,

    // Kept alive for the process lifetime; dropping it stops the device.
    device: Option<odin1::Device>,
}

impl OdinModule {
    async fn start(&mut self) {
        let opts = self.config.to_connect_options();
        let (device, frames) = match odin1::Device::connect(&opts) {
            Ok(pair) => pair,
            Err(err) => {
                tracing::error!("odin1 connect failed: {err}");
                std::process::exit(2);
            }
        };
        self.device = Some(device);

        let lidar = self.lidar.clone();
        let slam_cloud = self.slam_cloud.clone();
        let color_image = self.color_image.clone();
        let odometry = self.odometry.clone();
        let cfg = convert::PublishConfig::from(&self.config);
        let handle = tokio::runtime::Handle::current();

        // The SDK delivers frames on its own threads via a bounded channel.
        // Convert and publish on a dedicated thread; publish() only enqueues, so
        // block_on here is cheap and never stalls the runtime workers.
        std::thread::spawn(move || {
            for frame in frames.iter() {
                match frame {
                    Frame::Dtof(d) => {
                        let msg = convert::dtof_to_pointcloud(&d, &cfg);
                        let _ = handle.block_on(lidar.publish(&msg));
                    }
                    Frame::SlamCloud(c) => {
                        let msg = convert::slam_cloud_to_pointcloud(&c, &cfg);
                        let _ = handle.block_on(slam_cloud.publish(&msg));
                    }
                    Frame::Rgb(r) => {
                        let msg = convert::rgb_to_image(&r, &cfg);
                        let _ = handle.block_on(color_image.publish(&msg));
                    }
                    Frame::Odometry(o) => {
                        let msg = convert::odom_to_odometry(&o, &cfg);
                        let _ = handle.block_on(odometry.publish(&msg));
                    }
                    Frame::Imu(_) => {}
                }
            }
            tracing::info!("odin1 frame stream ended");
        });

        tracing::info!(mode = %self.config.mode, "odin1 module started");
    }
}

#[tokio::main]
async fn main() {
    let transport = LcmTransport::new()
        .await
        .expect("Failed to create transport");
    run::<OdinModule, _>(transport).await;
}
