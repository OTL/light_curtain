//
// @author Takashi Ogura <t.ogura@gmail.com>
//
// new BSD license
//

#ifndef LIGHT_CURTAIN_FORWARD_VELOCITY_FILTER_H
#define LIGHT_CURTAIN_FORWARD_VELOCITY_FILTER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <string>

namespace light_curtain {

class ForwardVelocityFilter {
 public:
  ForwardVelocityFilter(ros::NodeHandle& node,
                        const std::string& input_name,
                        const std::string& output_name,
                        const int32_t buffer_size);

  void callback(const geometry_msgs::Twist::ConstPtr& msg);

  void set_danger(bool is_danger);
 private:
  bool is_danger_;  ///< if this is danger, do not publish
  ros::NodeHandle node_;  ///< ROS node handle
  ros::Publisher velocity_publisher_;  ///< ros velocity topic publisher
  ros::Publisher state_publisher_;  ///< ros danger state publisher
  ros::Subscriber velocity_subscriber_;  ///< ros velocity topic subscriber
};

}  // namespace light_curtain


#endif  // LIGHT_CURTAIN_VELOCITY_FILTER_H
