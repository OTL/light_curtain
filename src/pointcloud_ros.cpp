#include <light_curtain/pointcloud_ros.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>

namespace {
const int32_t POINTCLOUD_BUFFER_SIZE = 1;
}


namespace light_curtain {

PointCloudROS::PointCloudROS(
    const ros::NodeHandle& node,
    const std::string& topic_name,
    const std::string& base_frame_id,
    const PointCloudCallback& callback)
    : node_(node)
    , topic_name_(topic_name)
    , base_frame_id_(base_frame_id)
    , callback_(callback)
{
}

void PointCloudROS::getPointCloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {

  sensor_msgs::PointCloud2 transformed_msg;
  if(tf_listener_->waitForTransform(base_frame_id_,
				    cloud_msg->header.frame_id,
				    cloud_msg->header.stamp,
				    ros::Duration(0.1))) {
    if (pcl_ros::transformPointCloud(base_frame_id_,
				     *cloud_msg,
				     transformed_msg,
				     *tf_listener_)) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(transformed_msg, *cloud);
      callback_(cloud);
    }
  }
}


void PointCloudROS::init() {
  pointcloud_subscriber_ = node_.subscribe(topic_name_,
                                           POINTCLOUD_BUFFER_SIZE,
                                           &PointCloudROS::getPointCloudCallback,
                                           this);
  tf_listener_ = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener(node_));
}

}  // namespace light_curtain


