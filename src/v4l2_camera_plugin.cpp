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
#include <cstring>
#include <iostream>

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

  // 2. Get Camera Topic
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
  
  // 3. Subscribe
  if (!camera_topic_.empty())
  {
    gz_node_.Subscribe(camera_topic_, &V4L2CameraPlugin::OnImage, this);
    gzmsg << "[V4L2Plugin] Subscribed to topic: " << camera_topic_ << " -> feeding to " << device_name_ << std::endl;
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

  struct v4l2_format vid_format;
  memset(&vid_format, 0, sizeof(vid_format));
  vid_format.type = V4L2_BUF_TYPE_VIDEO_OUTPUT; // Loopback is Output from our perspective (we write to it)
  
  // Note: some loopback devices behave as CAPTURE but allow write. 
  // However standard usage for feeding is writing to it.
  // v4l2loopback supports V4L2_BUF_TYPE_VIDEO_OUTPUT.

  vid_format.fmt.pix.width = width_;
  vid_format.fmt.pix.height = height_;
  vid_format.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24; // Gazebo usually sends RGB888
  vid_format.fmt.pix.sizeimage = width_ * height_ * 3;
  vid_format.fmt.pix.field = V4L2_FIELD_NONE;
  vid_format.fmt.pix.colorspace = V4L2_COLORSPACE_SRGB;

  if (ioctl(video_fd_, VIDIOC_S_FMT, &vid_format) == -1)
  {
    gzerr << "[V4L2Plugin] Failed to set format on " << device_name_ << ". Error: " << strerror(errno) << std::endl;
    close(video_fd_);
    return -1;
  }

  gzmsg << "[V4L2Plugin] Initialized " << device_name_ << " with " << width_ << "x" << height_ << " RGB24." << std::endl;
  return 0;
}

void V4L2CameraPlugin::OnImage(const gz::msgs::Image &_msg)
{
  if (!v4l2_initialized_)
  {
    width_ = _msg.width();
    height_ = _msg.height();
    
    // Check pixel format
    // Gazebo usually sends logic pixel format 3 (RGB_INT8)
    // We assume RGB888 for now.
    
    if (InitV4L2() == 0)
    {
      v4l2_initialized_ = true;
    }
    else
    {
      // Retry next time or stop? Retry might flood logs but safer to soft fail
      return; 
    }
  }

  if (video_fd_ >= 0)
  {
    // Write raw data
    // _msg.data() is string, need char ptr
    ssize_t written = write(video_fd_, _msg.data().c_str(), _msg.data().size());
    if (written < 0)
    {
       // Optionally log error (throttled)
    }
  }
}

} // namespace v4l2_gz_plugin

GZ_ADD_PLUGIN(v4l2_gz_plugin::V4L2CameraPlugin,
              gz::sim::System,
              v4l2_gz_plugin::V4L2CameraPlugin::ISystemConfigure,
              v4l2_gz_plugin::V4L2CameraPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(v4l2_gz_plugin::V4L2CameraPlugin, "v4l2_gz_plugin")
