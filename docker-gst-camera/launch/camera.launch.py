"""GPU-accelerated MJPEG camera publisher.

Launches gscam2 with one of two GStreamer pipelines:

  GPU (use_gpu:=true, default):
    v4l2src -> mjpeg caps -> nvjpegdec -> nvvideoconvert -> BGR
    Decode + convert run on the GPU. Final copy to system memory happens at
    gscam2's internal appsink. ~5% of frame time on the copy; the decode
    itself is essentially free.

  CPU (use_gpu:=false):
    v4l2src -> mjpeg caps -> jpegdec (libjpeg-turbo) -> videoconvert -> BGR
    Same pipeline structure, no NVIDIA dependency. Use on a laptop without
    an NVIDIA GPU, or for headless CI testing.

Topics published:
   /image_raw                   sensor_msgs/Image          (BGR8, 30 Hz)
   /image_raw/compressed        sensor_msgs/CompressedImage (from image_transport)
   /camera_info                 sensor_msgs/CameraInfo     (from camera_info_url)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Both pipelines end with `tee name=t ! queue ! fpsdisplaysink ...  t. ! queue`.
# The trailing `t. ! queue` is the branch gscam2 appends its appsink to (so
# ROS publishing still works); the other branch drains into fpsdisplaysink
# which prints `current: X.XX  average: X.XX` to stderr every second. That
# lets us measure the actual capture-pipeline rate independently of any
# ROS-side throttling (DDS QoS, `ros2 topic hz` mismeasurement, etc.).
_FPS_BRANCH = (
    "tee name=t ! queue ! "
    "fpsdisplaysink video-sink=fakesink text-overlay=false sync=false "
    "  t. ! queue"
)

GPU_PIPELINE = (
    "v4l2src device={device} ! "
    "image/jpeg,width={w},height={h},framerate={fps}/1 ! "
    "nvjpegdec ! "                    # nvjpeg-backed MJPEG decode, output NVMM
    "nvvideoconvert ! "               # GPU NV12 → RGB, then transfer to system mem
    "video/x-raw,format=RGB ! "       # system memory (NVMM tag dropped) so
                                      # gscam2's appsink can consume it
    + _FPS_BRANCH
)

CPU_PIPELINE = (
    "v4l2src device={device} ! "
    "image/jpeg,width={w},height={h},framerate={fps}/1 ! "
    "jpegdec ! "
    "videoconvert ! "
    "video/x-raw,format=RGB ! "
    + _FPS_BRANCH
)


def _setup(ctx, *_):
    use_gpu = LaunchConfiguration("use_gpu").perform(ctx).lower() in ("true", "1", "yes")
    device  = LaunchConfiguration("device").perform(ctx)
    width   = int(LaunchConfiguration("width").perform(ctx))
    height  = int(LaunchConfiguration("height").perform(ctx))
    fps     = int(LaunchConfiguration("fps").perform(ctx))

    pipeline = (GPU_PIPELINE if use_gpu else CPU_PIPELINE).format(
        device=device, w=width, h=height, fps=fps)
    print(f"[camera.launch] use_gpu={use_gpu}  pipeline:\n  {pipeline}")

    return [
        Node(
            package="gscam2",
            executable="gscam_main",
            name="camera",
            output="screen",
            emulate_tty=True,           # forces line-buffered stdout/stderr so
                                        # fpsdisplaysink's GST_INFO lines flush
                                        # promptly to `docker compose up`
            parameters=[
                "/opt/camera/config/camera.yaml",
                {"gscam_config": pipeline},
            ],
            additional_env={
                # fpsdisplaysink emits its rate measurement via GST_INFO; we
                # enable the GStreamer debug system at INFO for that one
                # element so the lines actually print. Color codes off so
                # the docker logs don't get peppered with ANSI escapes.
                "GST_DEBUG": "fpsdisplaysink:5",
                "GST_DEBUG_NO_COLOR": "1",
            },
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_gpu", default_value="true",
            description="true: nvjpegdec/nvvideoconvert; false: jpegdec/videoconvert"),
        DeclareLaunchArgument("device", default_value="/dev/video4"),
        DeclareLaunchArgument("width",  default_value="1280"),
        DeclareLaunchArgument("height", default_value="720"),
        DeclareLaunchArgument("fps",    default_value="30"),
        OpaqueFunction(function=_setup),
    ])
