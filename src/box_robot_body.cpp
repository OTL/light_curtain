//
// @author Takashi Ogura <t.ogura@gmail.com>
//
// new BSD license
//

#include <light_curtain/box_robot_body.h>
#include <pcl/common/common.h>
#include <pcl/ros/conversions.h>

namespace light_curtain {

BoxRobotBody::BoxRobotBody (const Eigen::Vector4f& min_point,
                            const Eigen::Vector4f& max_point)
    : min_point_(min_point)
    , max_point_(max_point)
{
}

bool BoxRobotBody::isNearBody(const sensor_msgs::PointCloud2& cloud_msg)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(cloud_msg, cloud);
  std::vector<int> indices;
  pcl::getPointsInBox(cloud, min_point_, max_point_, indices);
  return indices.size() > 0;
}

}  // namespace light_curtain
