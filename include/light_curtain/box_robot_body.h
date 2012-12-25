//
// @author Takashi Ogura <t.ogura@gmail.com>
//
// new BSD license
//

#ifndef LIGHT_CURTAIN_BOX_ROBOT_BODY_H
#define LIGHT_CURTAIN_BOX_ROBOT_BODY_H

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace light_curtain {

/// box type robot model
///
/// deside if the point cloud is inside of this model.
class BoxRobotBody {
 public:
  BoxRobotBody (const Eigen::Vector4f& min_point,
                const Eigen::Vector4f& max_point);

  bool isNearBody(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

  const Eigen::Vector4f& getMinPoint() const {
    return min_point_;
  }

  const Eigen::Vector4f& getMaxPoint() const {
    return max_point_;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  Eigen::Vector4f min_point_;
  Eigen::Vector4f max_point_;
};

}  // namespace light_curtain

#endif  // LIGHT_CURTAIN_BOX_ROBOT_BODY_H
