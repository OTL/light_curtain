//
// @author Takashi Ogura <t.ogura@gmail.com>
//
// new BSD license
//

#ifndef LIGHT_CURTAIN_LIGHT_CURTAIN_H
#define LIGHT_CURTAIN_LIGHT_CURTAIN_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <boost/function.hpp>

namespace light_curtain {

typedef boost::function<bool (const sensor_msgs::PointCloud2&)> JudgeDanger;
typedef boost::function<void (bool)> InformFunction;

class LightCurtain {
 public:
  LightCurtain(const ros::NodeHandle& node,
               const std::string& base_frame_id,
               const JudgeDanger& judge_callback,
               const InformFunction& inform_callback,
               const ros::Duration& keep_duration);
  bool isDanger() const;
  void init();
  void getLaserCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

 private:
  ros::NodeHandle node_;
  std::string base_frame_id_;
  JudgeDanger judge_callback_;
  InformFunction inform_callback_;
  ros::Duration keep_duration_;
  bool is_danger_;
  ros::Time last_danger_stamp_;
  ros::Subscriber laser_subscriber_;
  tf::TransformListener tf_listener_;

};

}

#endif  // LIGHT_CURTAIN_LIGHT_CURTAIN_H
