#ifndef LIGHT_CURTAIN_LASER_ROS_H
#define LIGHT_CURTAIN_LASER_ROS_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <boost/function.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/LaserScan.h>
#include <light_curtain/types.h>

namespace light_curtain {

class LaserROS {
 public:
  LaserROS(const ros::NodeHandle& node,
           const std::string& topic_name,
           const std::string& base_frame_id,
           const PointCloudCallback& callback);
  void init();
  void getLaserCallback(
      const sensor_msgs::LaserScan::ConstPtr& scan_msg);

 private:
  ros::NodeHandle node_;
  std::string topic_name_;
  ros::Subscriber laser_subscriber_;
  std::string base_frame_id_;
  boost::shared_ptr<tf::TransformListener> tf_listener_;
  PointCloudCallback callback_;
};

}  // light_curtain



#endif  // LIGHT_CURTAIN_LASER_POINTCLOUD_ROS_H
