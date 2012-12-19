//
// @author Takashi Ogura <t.ogura@gmail.com>
//
// new BSD license
//

#include <light_curtain/light_curtain.h>
#include <light_curtain/box_robot_body.h>
#include <geometry_msgs/Twist.h>

#include <boost/bind.hpp>

namespace light_curtain {

class ForwardVelocityFilter {
 public:
  ForwardVelocityFilter(ros::NodeHandle& node,
                        const std::string& input_name,
                        const std::string& output_name,
                        const int32_t buffer_size)
      : is_danger_(false)
      , node_(node){
    publisher_ = node_.advertise<geometry_msgs::Twist>(output_name, buffer_size);
    subscriber_ = node_.subscribe(input_name, buffer_size,
                                  &ForwardVelocityFilter::callback, this);
  }

  void callback(const geometry_msgs::Twist::ConstPtr& msg) {
    if (is_danger_ && (msg->linear.x > 0.0)) {
      // do not publish
    } else {
      publisher_.publish(msg);
    }
  }

  void set_danger(bool is_danger) {
    is_danger_ = is_danger;
  }
 private:
  bool is_danger_;  ///< if this is danger, do not publish
  ros::NodeHandle node_;  ///< ROS node handle
  ros::Publisher publisher_;  ///< ros topic publisher
  ros::Subscriber subscriber_;  ///< ros topic subscriber
};

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "light_curtain_velocity_filter");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");

  double width(0.5);
  private_node.param("robot_width", width, 0.5);
  double depth(0.5);
  private_node.param("robot_depth", depth, 0.5);
  double height(1.5);
  private_node.param("robot_height", height, 1.5);
  double keep_duration(1.0);
  private_node.param("keep_duration", keep_duration, 1.0);

  std::string base_frame("base_link");
  private_node.param<std::string>("base_frame", base_frame, "base_link");

  using light_curtain::ForwardVelocityFilter;
  using light_curtain::BoxRobotBody;
  using Eigen::Vector4f;
  using light_curtain::LightCurtain;

  BoxRobotBody robot(Vector4f(-depth/2, -width/2, 0.0, 0.0),
                     Vector4f(depth/2, width/2, height, 0.0));

  ForwardVelocityFilter filter(node,
                               "input_velocity",
                               "cmd_vel",
                               10);

  LightCurtain curtain(
      node,
      base_frame,
      boost::bind(&BoxRobotBody::isNearBody, &robot, _1),
      boost::bind(&ForwardVelocityFilter::set_danger, &filter, _1),
      ros::Duration(keep_duration));
  curtain.init();
  ros::spin();

  return 0;
}
