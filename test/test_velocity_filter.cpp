#include <gtest/gtest.h>
#include <ros/ros.h>
#include <light_curtain/velocity_filter.h>

using light_curtain::ForwardVelocityFilter;

class TwistSubscriber {
 public:
  TwistSubscriber(ros::NodeHandle& node, const std::string& topic_name) {
    sub_ = node.subscribe(topic_name, 1, &TwistSubscriber::callback, this);
    data_.linear.x = -1.0;
  }
  void callback(const geometry_msgs::Twist::ConstPtr& msg) {
    data_ = *msg;
  }
  const geometry_msgs::Twist& getData() {
    return data_;
  }
 private:
  ros::Subscriber sub_;
  geometry_msgs::Twist data_;
};

TEST(VelocityFilter, forward) {
  ros::NodeHandle node;
  TwistSubscriber sub(node, "out");
  ForwardVelocityFilter filter(node, "in", "out", 1);
  ros::Duration(2.0).sleep();
  // default false
  geometry_msgs::Twist::Ptr msg(new geometry_msgs::Twist());
  msg->linear.x = 2.0;
  filter.callback(msg);
  ros::Duration(1.0).sleep();
  ros::spinOnce();
  EXPECT_NEAR(2.0, sub.getData().linear.x, 0.000001);

  // enable
  filter.set_danger(true);
  msg->linear.x = 3.0;
  filter.callback(msg);
  ros::Duration(1.0).sleep();
  ros::spinOnce();
  EXPECT_NEAR(2.0, sub.getData().linear.x, 0.000001);

  // disable
  filter.set_danger(false);
  msg->linear.x = 5.0;
  filter.callback(msg);
  ros::Duration(1.0).sleep();
  ros::spinOnce();
  EXPECT_NEAR(5.0, sub.getData().linear.x, 0.000001);
}

TEST(VelocityFilter, backward) {
  ros::NodeHandle node;
  TwistSubscriber sub(node, "out");
  ForwardVelocityFilter filter(node, "in", "out", 1);
  ros::Duration(2.0).sleep();
  // default false
  geometry_msgs::Twist::Ptr msg(new geometry_msgs::Twist());
  msg->linear.x = -2.0;
  filter.callback(msg);
  ros::Duration(1.0).sleep();
  ros::spinOnce();
  EXPECT_NEAR(-2.0, sub.getData().linear.x, 0.000001);

  // enable
  filter.set_danger(true);
  msg->linear.x = -3.0;
  filter.callback(msg);
  ros::Duration(1.0).sleep();
  ros::spinOnce();
  EXPECT_NEAR(-3.0, sub.getData().linear.x, 0.000001);

  // disable
  filter.set_danger(false);
  msg->linear.x = -5.0;
  filter.callback(msg);
  ros::Duration(1.0).sleep();
  ros::spinOnce();
  EXPECT_NEAR(-5.0, sub.getData().linear.x, 0.000001);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "test_velocity_filter");

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
