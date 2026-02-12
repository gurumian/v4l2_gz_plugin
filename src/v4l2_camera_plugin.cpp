#include "v4l2_gz_plugin/v4l2_camera_plugin.hpp"

#include <gz/plugin/Register.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/sim/Util.hh>
#include <gz/common/Console.hh>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <algorithm>

namespace v4l2_gz_plugin
{

V4L2CameraPlugin::V4L2CameraPlugin()
{
}

V4L2CameraPlugin::~V4L2CameraPlugin()
{
  if (video_fd_ >= 0)
  {
    close(video_fd_);
    video_fd_ = -1;
  }
}

void V4L2CameraPlugin::Configure(const gz::sim::Entity &_entity,
                                 const std::shared_ptr<const sdf::Element> &_sdf,
                                 gz::sim::EntityComponentManager &_ecm,
                                 gz::sim::EventManager &/*_eventMgr*/)
{
  // 1. Get Device Name from SDF
  if (_sdf->HasElement("device"))
    device_name_ = _sdf->Get<std::string>("device");
  else
    device_name_ = "/dev/video10";

  // 2. Pixel format for V4L2 output: "rgb24" (default) or "yuyv"
  if (_sdf->HasElement("pixel_format"))
  {
    pixel_format_ = _sdf->Get<std::string>("pixel_format");
    if (pixel_format_ != "rgb24" && pixel_format_ != "yuyv")
    {
      gzerr << "[V4L2Plugin] Unknown pixel_format '" << pixel_format_ << "', using rgb24." << std::endl;
      pixel_format_ = "rgb24";
    }
  }

  // 3. Get Camera Topic
  // Prefer manual topic if provided, else auto-generate based on entity
  if (_sdf->HasElement("topic"))
  {
    camera_topic_ = _sdf->Get<std::string>("topic");
  }
  else
  {
    // Auto-detect topic: Use scoped name of the entity
    // The standard default topic for a camera sensor is usually its scoped name (plus /image sometimes)
    // Actually, Gazebo Sim usually publishes on `scoped_name/image` or just `scoped_name` depending on config.
    // Let's rely on standard convention: /world/<world_name>/model/<model_name>/link/<link_name>/sensor/<sensor_name>/image
    
    std::string name = gz::sim::scopedName(_entity, _ecm, "/", false);
    camera_topic_ = name + "/image";
    
    gzmsg << "[V4L2Plugin] No 'topic' param provided. Auto-detected topic: " << camera_topic_ << std::endl;
  }
  
  // 4. Subscribe
  if (!camera_topic_.empty())
  {
    gz_node_.Subscribe(camera_topic_, &V4L2CameraPlugin::OnImage, this);
    gzmsg << "[V4L2Plugin] Subscribed to topic: " << camera_topic_ << " -> feeding to " << device_name_
          << " (" << pixel_format_ << ")" << std::endl;
  }
}

void V4L2CameraPlugin::PostUpdate(const gz::sim::UpdateInfo &/*_info*/,
                                  const gz::sim::EntityComponentManager &/*_ecm*/)
{
  // No-op
}

int V4L2CameraPlugin::InitV4L2()
{
  video_fd_ = open(device_name_.c_str(), O_RDWR);
  if (video_fd_ < 0)
  {
    gzerr << "[V4L2Plugin] Failed to open device: " << device_name_ << " (Error: " << strerror(errno) << ")" << std::endl;
    return -1;
  }

  struct v4l2_capability vid_caps;
  if (ioctl(video_fd_, VIDIOC_QUERYCAP, &vid_caps) == -1)
  {
    gzerr << "[V4L2Plugin] Failed to query capabilities." << std::endl;
    close(video_fd_);
    return -1;
  }

  if (!(vid_caps.capabilities & V4L2_CAP_VIDEO_OUTPUT))
  {
    gzerr << "[V4L2Plugin] " << device_name_ << " does not support VIDEO_OUTPUT (writer side). "
          << "This is required for feeding frames. Real webcams are capture-only. "
          << "Use a v4l2loopback device instead, e.g.: sudo modprobe v4l2loopback video_nr=10"
          << std::endl;
    close(video_fd_);
    video_fd_ = -1;
    return -1;
  }

  struct v4l2_format vid_format;
  memset(&vid_format, 0, sizeof(vid_format));
  vid_format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT; // Loopback is Output from our perspective (we write to it)
  
  // Note: some loopback devices behave as CAPTURE but allow write. 
  // However standard usage for feeding is writing to it.
  // v4l2loopback supports V4L2_BUF_TYPE_VIDEO_OUTPUT.

  vid_format.fmt.pix.width = width_;
  vid_format.fmt.pix.height = height_;
  bool use_yuyv = (pixel_format_ == "yuyv");

  auto try_set_format = [this, &vid_format](uint32_t fourcc, int bytes_per_pixel) -> bool {
    vid_format.fmt.pix.pixelformat = fourcc;
    vid_format.fmt.pix.sizeimage = width_ * height_ * bytes_per_pixel;
    vid_format.fmt.pix.field = V4L2_FIELD_NONE;
    vid_format.fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;
    return ioctl(video_fd_, VIDIOC_S_FMT, &vid_format) == 0;
  };

  if (use_yuyv && try_set_format(V4L2_PIX_FMT_YUYV, 2))
  {
    yuyv_buffer_.resize(static_cast<size_t>(width_ * height_ * 2));
    gzmsg << "[V4L2Plugin] Initialized " << device_name_ << " with " << width_ << "x" << height_
          << " yuyv." << std::endl;
    return 0;
  }
  if (use_yuyv && errno == EINVAL)
  {
    gzmsg << "[V4L2Plugin] Device does not support YUYV, falling back to RGB24." << std::endl;
    use_yuyv = false;
    pixel_format_ = "rgb24";
  }

  if (!use_yuyv && try_set_format(V4L2_PIX_FMT_RGB24, 3))
  {
    gzmsg << "[V4L2Plugin] Initialized " << device_name_ << " with " << width_ << "x" << height_
          << " rgb24." << std::endl;
    return 0;
  }

  gzerr << "[V4L2Plugin] Failed to set format on " << device_name_ << ". Error: " << strerror(errno)
        << ". Possible causes: (1) Device is not v4l2loopback — use: sudo modprobe v4l2loopback video_nr=10. "
        << "(2) Another app already set a different format — close it and restart Gazebo. "
        << "(3) Reload loopback for a clean state: sudo modprobe -r v4l2loopback && sudo modprobe v4l2loopback video_nr=10"
        << std::endl;
  close(video_fd_);
  video_fd_ = -1;
  return -1;
}

void V4L2CameraPlugin::RgbToYuyv(const uint8_t * _rgb, int _width, int _height, uint8_t * _yuyv)
{
  // BT.601: Y = (77*R + 150*G + 29*B)>>8, U = ((-43*R - 84*G + 127*B)>>8)+128, V = ((127*R - 106*G - 21*B)>>8)+128
  // YUYV: for each pair of pixels (R0,G0,B0)(R1,G1,B1) -> Y0 U Y1 V (U,V averaged)
  for (int y = 0; y < _height; ++y)
  {
    const uint8_t * row = _rgb + static_cast<size_t>(y * _width * 3);
    uint8_t * out = _yuyv + static_cast<size_t>(y * _width * 2);
    for (int x = 0; x < _width; x += 2)
    {
      int r0 = row[x * 3 + 0], g0 = row[x * 3 + 1], b0 = row[x * 3 + 2];
      int r1 = (x + 1 < _width) ? row[(x + 1) * 3 + 0] : r0;
      int g1 = (x + 1 < _width) ? row[(x + 1) * 3 + 1] : g0;
      int b1 = (x + 1 < _width) ? row[(x + 1) * 3 + 2] : b0;
      int y0 = ((77 * r0 + 150 * g0 + 29 * b0) >> 8);
      int y1 = ((77 * r1 + 150 * g1 + 29 * b1) >> 8);
      int u0 = ((-43 * r0 - 84 * g0 + 127 * b0) >> 8) + 128;
      int v0 = ((127 * r0 - 106 * g0 - 21 * b0) >> 8) + 128;
      int u1 = ((-43 * r1 - 84 * g1 + 127 * b1) >> 8) + 128;
      int v1 = ((127 * r1 - 106 * g1 - 21 * b1) >> 8) + 128;
      out[0] = static_cast<uint8_t>(std::clamp(y0, 0, 255));
      out[1] = static_cast<uint8_t>(std::clamp((u0 + u1) / 2, 0, 255));
      out[2] = static_cast<uint8_t>(std::clamp(y1, 0, 255));
      out[3] = static_cast<uint8_t>(std::clamp((v0 + v1) / 2, 0, 255));
      out += 4;
    }
  }
}

void V4L2CameraPlugin::OnImage(const gz::msgs::Image &_msg)
{
  if (!v4l2_initialized_)
  {
    width_ = _msg.width();
    height_ = _msg.height();

    if (InitV4L2() == 0)
      v4l2_initialized_ = true;
    else
      return;
  }

  if (video_fd_ >= 0)
  {
    const char * data = _msg.data().c_str();
    size_t data_size = _msg.data().size();
    if (pixel_format_ == "yuyv" && data_size == static_cast<size_t>(width_ * height_ * 3))
    {
      RgbToYuyv(reinterpret_cast<const uint8_t *>(data), width_, height_, yuyv_buffer_.data());
      write(video_fd_, yuyv_buffer_.data(), yuyv_buffer_.size());
    }
    else if (data_size == static_cast<size_t>(width_ * height_ * 3))
      write(video_fd_, data, data_size);
  }
}

} // namespace v4l2_gz_plugin

GZ_ADD_PLUGIN(v4l2_gz_plugin::V4L2CameraPlugin,
              gz::sim::System,
              v4l2_gz_plugin::V4L2CameraPlugin::ISystemConfigure,
              v4l2_gz_plugin::V4L2CameraPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(v4l2_gz_plugin::V4L2CameraPlugin, "v4l2_gz_plugin")
