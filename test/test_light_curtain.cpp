#include <gtest/gtest.h>
#include <ros/ros.h>
#include <light_curtain/light_curtain.h>

using light_curtain::LightCurtain;

namespace {
bool g_informed_value = false;
int32_t g_called_count = 0;
}

bool isDanger(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
  return cloud->size() > 0;
}

void inform(bool val) {
  ++g_called_count;
  g_informed_value = val;
}


TEST(LightCurtain, setDanger) {
  LightCurtain curtain(isDanger,
                       inform,
                       ros::Duration(1.0));
  // default safe
  EXPECT_FALSE(curtain.isDanger());
  curtain.setDanger(true);
  EXPECT_TRUE(curtain.isDanger());
  curtain.setDanger(false);
  EXPECT_FALSE(curtain.isDanger());
}

TEST(LightCurtain, updatePointCloud) {
  LightCurtain curtain(isDanger,
                       inform,
                       ros::Duration(1.0));
  curtain.init();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  // default false -> false
  g_informed_value = false;
  g_called_count = 0;

  curtain.updatePointCloud(cloud);
  EXPECT_FALSE(g_informed_value);
  EXPECT_EQ(0, g_called_count);

  pcl::PointXYZ p1;
  p1.x = 0.2;
  p1.y = 0.3;
  p1.z = 0.4;
  cloud->push_back(p1);

  // danger -> true
  curtain.updatePointCloud(cloud);
  EXPECT_TRUE(g_informed_value);
  EXPECT_EQ(1, g_called_count);

  // safe but time is not passed yet
  cloud->clear();
  curtain.updatePointCloud(cloud);
  EXPECT_TRUE(g_informed_value);
  EXPECT_EQ(1, g_called_count);

  ros::Duration(1.5).sleep();

  // safe and time has passed
  curtain.updatePointCloud(cloud);
  EXPECT_FALSE(g_informed_value);
  EXPECT_EQ(2, g_called_count);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "test_light_curtain");
  ros::NodeHandle node;
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
