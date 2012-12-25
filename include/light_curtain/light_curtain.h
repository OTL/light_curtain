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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <light_curtain/types.h>

namespace light_curtain {

class LightCurtain {
 public:
  LightCurtain(const JudgeDanger& judge_callback,
               const InformFunction& inform_callback,
               const ros::Duration& keep_duration);
  bool isDanger() const;
  void init();
  void updatePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  void setDanger(bool is_danger);
 private:
  JudgeDanger judge_callback_;
  InformFunction inform_callback_;
  ros::Duration keep_duration_;
  bool is_danger_;
  ros::Time last_danger_stamp_;
};

}

#endif  // LIGHT_CURTAIN_LIGHT_CURTAIN_H
