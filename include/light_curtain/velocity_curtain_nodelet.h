#ifndef LIGHT_CURTAIN_VELOCITY_CURTAIN_NODELET_H
#define LIGHT_CURTAIN_VELOCITY_CURTAIN_NODELET_H

#include <nodelet/nodelet.h>
#include <dynamic_reconfigure/server.h>

#include <light_curtain/VelocityCurtainConfig.h>
#include <light_curtain/box_robot_body.h>
#include <light_curtain/light_curtain.h>
#include <light_curtain/pointcloud_ros.h>
#include <light_curtain/laser_ros.h>
#include <light_curtain/velocity_filter.h>



namespace light_curtain {

class VelocityCurtainNodelet : public nodelet::Nodelet
{
 public:
  VelocityCurtainNodelet();
  virtual void onInit();
  virtual void reconfigureCallback(VelocityCurtainConfig& config, uint32_t level);
  virtual ~VelocityCurtainNodelet();
 private:
  typedef dynamic_reconfigure::Server<VelocityCurtainConfig> ConfigServer;
  boost::shared_ptr<BoxRobotBody> robot_;
  boost::shared_ptr<ForwardVelocityFilter> filter_;
  boost::shared_ptr<LightCurtain> curtain_;
  boost::shared_ptr<PointCloudROS> pointcloud_updater_;
  boost::shared_ptr<LaserROS> laser_updater_;
  boost::shared_ptr<ConfigServer> config_server_;
};

}  // namespace light_curtain

#endif  // LIGHT_CURTAIN_VELOCITY_CURTAIN_NODELET_H
