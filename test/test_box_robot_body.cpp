#include <gtest/gtest.h>
#include <ros/ros.h>
#include <light_curtain/box_robot_body.h>

using light_curtain::BoxRobotBody;
using Eigen::Vector4f;

TEST(BoxRobotBody, BoxRobotBody) {
  Vector4f v1(0.1, 0.2, 0.3, 0.0);
  Vector4f v2(1.0, 1.1, 1.2, 0.0);
  BoxRobotBody b(v1, v2);
  EXPECT_NEAR(0.1, b.getMinPoint()(0), 0.000001);
  EXPECT_NEAR(0.2, b.getMinPoint()(1), 0.000001);
  EXPECT_NEAR(0.3, b.getMinPoint()(2), 0.000001);
  EXPECT_NEAR(1.0, b.getMaxPoint()(0), 0.000001);
  EXPECT_NEAR(1.1, b.getMaxPoint()(1), 0.000001);
  EXPECT_NEAR(1.2, b.getMaxPoint()(2), 0.000001);
}

TEST(BoxRobotBody, setMinPoint) {
  Vector4f v1(0.1, 0.2, 0.3, 0.0);
  Vector4f v2(1.0, 1.1, 1.2, 0.0);
  BoxRobotBody b(v1, v2);
  Vector4f v3(-0.1, -0.2, -0.3, 0.0);
  b.setMinPoint(v3);
  EXPECT_NEAR(-0.1, b.getMinPoint()(0), 0.000001);
  EXPECT_NEAR(-0.2, b.getMinPoint()(1), 0.000001);
  EXPECT_NEAR(-0.3, b.getMinPoint()(2), 0.000001);
}


TEST(BoxRobotBody, setMaxPoint) {
  Vector4f v1(0.1, 0.2, 0.3, 0.0);
  Vector4f v2(1.0, 1.1, 1.2, 0.0);
  BoxRobotBody b(v1, v2);
  Vector4f v3(0.5, 0.8, 0.9, 0.0);
  b.setMinPoint(v3);
  EXPECT_NEAR(0.5, b.getMinPoint()(0), 0.000001);
  EXPECT_NEAR(0.8, b.getMinPoint()(1), 0.000001);
  EXPECT_NEAR(0.9, b.getMinPoint()(2), 0.000001);
}

TEST(BoxRobotBody, isCollided) {
  Vector4f v1(0.1, 0.2, 0.3, 0.0);
  Vector4f v2(1.0, 1.1, 1.2, 0.0);
  BoxRobotBody b(v1, v2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ p1;
  p1.x = 0.0;
  p1.y = 0.0;
  p1.z = 0.0;
  cloud->push_back(p1);
  EXPECT_FALSE(b.isCollided(cloud));
  pcl::PointXYZ p2;
  p2.x = 0.2;
  p2.y = 0.3;
  p2.z = 0.4;
  cloud->push_back(p2);

  EXPECT_TRUE(b.isCollided(cloud));
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "test_box_robot_body");

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
