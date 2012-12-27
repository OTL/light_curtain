//
// @author Takashi Ogura <t.ogura@gmail.com>
//
// new BSD license
//

#include <light_curtain/forward_velocity_filter.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

namespace {
const int32_t DANGER_STATE_BUFFER = 10;
}

namespace light_curtain {


ForwardVelocityFilter::ForwardVelocityFilter(
    ros::NodeHandle& node,
    const std::string& input_name,
    const std::string& output_name,
    const int32_t buffer_size)
    : is_danger_(false)
    , node_(node){
  velocity_publisher_ = node_.advertise<geometry_msgs::Twist>(output_name, buffer_size);
  velocity_subscriber_ = node_.subscribe(input_name, buffer_size,
                                &ForwardVelocityFilter::callback, this);
  state_publisher_ = node_.advertise<std_msgs::Bool>("/curtain/danger_state",
                                                     DANGER_STATE_BUFFER);

}

void ForwardVelocityFilter::callback(const geometry_msgs::Twist::ConstPtr& msg) {
  if (is_danger_ && (msg->linear.x > 0.0)) {
    // publish linear.x = 0.0
    geometry_msgs::Twist filtered_msg(*msg);
    filtered_msg.linear.x = 0.0;
    velocity_publisher_.publish(filtered_msg);
  } else {
    velocity_publisher_.publish(msg);
  }
  std_msgs::Bool danger_state;
  danger_state.data = is_danger_;
  state_publisher_.publish(danger_state);
}

void ForwardVelocityFilter::set_danger(bool is_danger) {
  is_danger_ = is_danger;
}

}  // namespace light_curtain

