# v4l2_gz_plugin

A Gazebo (Ignition) System Plugin that feeds camera sensor data directly to a V4L2 loopback device (e.g., `/dev/video10`). This allows Gazebo simulation cameras to be consumed by standard video applications like WebRTC, GStreamer, OpenCV, or web browsers.

## Prerequisites

You need `v4l2loopback` installed to create a virtual video device.

```bash
sudo apt-get install v4l2loopback-dkms
```

## Setup

Before launching the simulation, load the kernel module to create the video device.

```bash
# Create /dev/video10 with a specific label
sudo modprobe v4l2loopback video_nr=10 card_label="GzCamera" exclusive_caps=1
```

To make this permanent, you can add it to `/etc/modules`, or create a config in `/etc/modprobe.d/`.

## Build

Building as a standard ROS 2 package:

```bash
cd ~/workspace/turtlebot3-build
colcon build --packages-select v4l2_gz_plugin
source install/setup.bash
```

## Usage

Add the plugin to your robot's URDF/SDF file within the `<sensor>` tag. 

**Example (`camera.gazebo`):**

```xml
<gazebo reference="camera_rgb_optical_frame">
  <sensor name="camera" type="camera">
    <!-- Existing camera configuration -->
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>30</update_rate>
    <topic>camera/image_raw</topic>
    <camera name="intel_realsense_r200">
      <image>
        <format>R8G8B8</format>
        <width>1920</width>
        <height>1080</height>
      </image>
      <!-- ... other params ... -->
    </camera>

    <!-- v4l2_gz_plugin Configuration -->
    <plugin filename="libv4l2_gz_plugin.so" name="v4l2_gz_plugin::V4L2CameraPlugin">
      <!-- Target video device (must exist) -->
      <device>/dev/video10</device> 
      <!-- Topic to subscribe to (optional, auto-detected if omitted but recommended) -->
      <topic>camera/image_raw</topic>
      <pixel_format>yuyv</pixel_format>
    </plugin>
  </sensor>
</gazebo>
```

### Environment Variable
Ensure the plugin library is in the `GZ_SIM_SYSTEM_PLUGIN_PATH`. Sourcing the ROS workspace (`install/setup.bash`) usually handles this, but if not:

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:$(ros2 pkg prefix v4l2_gz_plugin)/lib/v4l2_gz_plugin
```

## Verification

Run the simulation and check the video feed using any video player:

```bash
# Using ffplay
ffplay /dev/video10

# Using mpv
mpv av://v4l2:/dev/video10

# Using GStreamer
gst-launch-1.0 v4l2src device=/dev/video10 ! autovideosink
```

## Notes

- **Pixel Format**: The plugin writes raw RGB24 data. Ensure the consumer application expects RGB.
- **Orientation**: If the image appears rotated, adjust the `<pose>` tag in the sensor configuration.
