//
// @author Takashi Ogura <t.ogura@gmail.com>
//
// new BSD license
//

#include <light_curtain/velocity_filter.h>
#include <geometry_msgs/Twist.h>

namespace light_curtain {

ForwardVelocityFilter::ForwardVelocityFilter(
    ros::NodeHandle& node,
    const std::string& input_name,
    const std::string& output_name,
    const int32_t buffer_size)
    : is_danger_(false)
    , node_(node){
  publisher_ = node_.advertise<geometry_msgs::Twist>(output_name, buffer_size);
  subscriber_ = node_.subscribe(input_name, buffer_size,
                                &ForwardVelocityFilter::callback, this);
}

void ForwardVelocityFilter::callback(const geometry_msgs::Twist::ConstPtr& msg) {
  if (is_danger_ && (msg->linear.x > 0.0)) {
    // do not publish
  } else {
    publisher_.publish(msg);
  }
}

void ForwardVelocityFilter::set_danger(bool is_danger) {
  is_danger_ = is_danger;
}

}  // namespace light_curtain

