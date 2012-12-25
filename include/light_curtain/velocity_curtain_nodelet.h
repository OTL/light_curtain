#ifndef LIGHT_CURTAIN_VELOCITY_CURTAIN_NODELET_H
#define LIGHT_CURTAIN_VELOCITY_CURTAIN_NODELET_H

#include <nodelet/nodelet.h>
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
  virtual ~VelocityCurtainNodelet();
 private:
  boost::shared_ptr<BoxRobotBody> robot_;
  boost::shared_ptr<ForwardVelocityFilter> filter_;
  boost::shared_ptr<LightCurtain> curtain_;
  boost::shared_ptr<PointCloudROS> pointcloud_updater_;
  boost::shared_ptr<LaserROS> laser_updater_;
};

}  // namespace light_curtain

#endif  // LIGHT_CURTAIN_VELOCITY_CURTAIN_NODELET_H
