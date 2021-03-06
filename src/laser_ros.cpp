#include <light_curtain/laser_ros.h>
#include <pcl/ros/conversions.h>
#include <laser_geometry/laser_geometry.h>

namespace {
const int32_t LASER_BUFFER_SIZE = 50;
}


namespace light_curtain {

LaserROS::LaserROS(
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

void LaserROS::getLaserCallback(
    const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
  laser_geometry::LaserProjection projector_;
  sensor_msgs::PointCloud2 cloud_msg;
  if(tf_listener_->waitForTransform(base_frame_id_,
				    scan_msg->header.frame_id,
				    scan_msg->header.stamp,
				    ros::Duration(0.1))) {
    try {
      projector_.transformLaserScanToPointCloud(base_frame_id_,
						*scan_msg,
						cloud_msg,
						*tf_listener_);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(cloud_msg, *cloud);
      callback_(cloud);
    } catch (tf::TransformException& e) {
      ROS_WARN_STREAM_ONCE("transform laser failed: " << e.what());
    }
  }
}


void LaserROS::init() {
  laser_subscriber_ = node_.subscribe(topic_name_,
                                      LASER_BUFFER_SIZE,
                                      &LaserROS::getLaserCallback,
                                      this);
  tf_listener_ = boost::shared_ptr<tf::TransformListener>(new tf::TransformListener(node_));
}

}  // namespace light_curtain


