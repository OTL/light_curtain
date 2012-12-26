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
               double keep_duration);
  void init();
  void updatePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
  bool getDangerState() const;
  void setDangerState(bool is_danger);
  void setKeepDuration(double duration) {
    keep_duration_ = duration;
  };
  double getKeepDuration() {
    return keep_duration_;
  };

 private:
  JudgeDanger judge_callback_;
  InformFunction inform_callback_;
  double keep_duration_;
  bool is_danger_;
  ros::Time last_danger_stamp_;
};

}

#endif  // LIGHT_CURTAIN_LIGHT_CURTAIN_H
