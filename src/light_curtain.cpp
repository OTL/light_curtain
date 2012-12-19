//
// @author Takashi Ogura <t.ogura@gmail.com>
//
// new BSD license
//

#include <light_curtain/light_curtain.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

namespace {
const int32_t LASER_BUFFER_SIZE = 50;
}

namespace light_curtain {

LightCurtain::LightCurtain(const ros::NodeHandle& node,
                           const std::string& base_frame_id,
                           const JudgeDanger& judge_callback,
                           const InformFunction& inform_callback,
                           const ros::Duration& keep_duration)
    : node_(node)
    , base_frame_id_(base_frame_id)
    , judge_callback_(judge_callback)
    , inform_callback_(inform_callback)
    , keep_duration_(keep_duration)
    , is_danger_(false)
{

}

bool LightCurtain::isDanger() const {
  return is_danger_;
}

void LightCurtain::init()
{
  laser_subscriber_ = node_.subscribe("/scan", LASER_BUFFER_SIZE,
                                      &LightCurtain::getLaserCallback, this);
  last_danger_stamp_ = ros::Time::now();
}

void LightCurtain::setDanger(bool is_danger) {
  if (is_danger_ != is_danger) {
    inform_callback_(is_danger);
  }
  is_danger_ = is_danger;
}

void LightCurtain::getLaserCallback(
    const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
  laser_geometry::LaserProjection projector_;
  sensor_msgs::PointCloud2 cloud;
  try {
    projector_.transformLaserScanToPointCloud(base_frame_id_,
                                              *scan_msg,
                                              cloud,
                                              tf_listener_);
  } catch (tf::TransformException& e) {
    ROS_WARN_STREAM_ONCE("transform laser failed: " << e.what());
  }
  if (judge_callback_(cloud)) {
    // danger
    if (scan_msg->header.stamp > last_danger_stamp_) {
      last_danger_stamp_ = scan_msg->header.stamp;
    }
    setDanger(true);
  } else if (scan_msg->header.stamp - last_danger_stamp_ > keep_duration_) {
    setDanger(false);
  }
}

}
