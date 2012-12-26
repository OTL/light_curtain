#ifndef LIGHT_CURTAIN_POINTCLOUD_ROS_H
#define LIGHT_CURTAIN_POINTCLOUD_ROS_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <boost/function.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <light_curtain/types.h>

namespace light_curtain {

class PointCloudROS {
 public:
  PointCloudROS(const ros::NodeHandle& node,
                const std::string& topic_name,
                const std::string& base_frame_id,
                const PointCloudCallback& callback);
  void init();
  void getPointCloudCallback(
      const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
  void setBaseFrameId(const std::string& base_frame_id) {
    base_frame_id_ = base_frame_id;
  }
  const std::string& getBaseFrameId() const {
    return base_frame_id_;
  }

 private:
  ros::NodeHandle node_;
  std::string topic_name_;
  ros::Subscriber pointcloud_subscriber_;
  std::string base_frame_id_;
  boost::shared_ptr<tf::TransformListener> tf_listener_;
  PointCloudCallback callback_;
};

}  // light_curtain



#endif  // LIGHT_CURTAIN_POINTCLOUD_ROS_H
