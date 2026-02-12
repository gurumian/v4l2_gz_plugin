#ifndef V4L2_GZ_PLUGIN_HPP_
#define V4L2_GZ_PLUGIN_HPP_

#include <gz/sim/System.hh>
#include <gz/sim/Entity.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/image.pb.h>

#include <memory>
#include <string>
#include <vector>

namespace v4l2_gz_plugin
{

class V4L2CameraPlugin : public gz::sim::System,
                         public gz::sim::ISystemConfigure,
                         public gz::sim::ISystemPostUpdate
{
public:
  V4L2CameraPlugin();
  ~V4L2CameraPlugin() override;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  void PostUpdate(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager &_ecm) override;

private:
  void OnImage(const gz::msgs::Image &_msg);
  int InitV4L2();
  void RgbToYuyv(const uint8_t * _rgb, int _width, int _height, uint8_t * _yuyv);

  gz::transport::Node gz_node_;
  std::string device_name_;
  std::string camera_topic_;
  std::string pixel_format_{"rgb24"};  // "rgb24" or "yuyv"
  std::vector<uint8_t> yuyv_buffer_;
  int width_ = 0;
  int height_ = 0;
  int video_fd_ = -1;
  bool v4l2_initialized_ = false;
};

} // namespace v4l2_gz_plugin

#endif // V4L2_GZ_PLUGIN_HPP_
