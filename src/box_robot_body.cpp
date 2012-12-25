//
// @author Takashi Ogura <t.ogura@gmail.com>
//
// new BSD license
//

#include <light_curtain/box_robot_body.h>
#include <pcl/common/common.h>

namespace light_curtain {

BoxRobotBody::BoxRobotBody (const Eigen::Vector4f& min_point,
                            const Eigen::Vector4f& max_point)
    : min_point_(min_point)
    , max_point_(max_point)
{
}

bool BoxRobotBody::isNearBody(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
  std::vector<int> indices;
  pcl::getPointsInBox(*cloud, min_point_, max_point_, indices);
  return indices.size() > 0;
}

}  // namespace light_curtain
