//
// @author Takashi Ogura <t.ogura@gmail.com>
//
// new BSD license
//

#include <light_curtain/light_curtain.h>
#include <light_curtain/box_robot_body.h>
#include <light_curtain/filter.h>
#include <geometry_msgs/Twist.h>

#include <boost/bind.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "light_curtain_velocity_filter");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");

  double width(0.5);
  private_node.param("robot_width", width, 0.5);
  double depth(0.5);
  private_node.param("robot_depth", depth, 0.5);
  double height(1.5);
  private_node.param("robot_height", height, 1.0);
  double keep_duration(1.0);
  private_node.param("keep_duration", keep_duration, 1.0);

  std::string base_farame("base_link");
  private_node.param("base_frame", base_link, "base_link");

  using light_curtain::Filter<geometry_msgs::Twist>;
  using light_curtain::BoxRobotBody;
  using Eigen::Vector4f;
  using light_curtain::LightCurtain;

  BoxRobotBody robot(Vector4f(-depth/2, -width/2, 0.0, 0.0),
                     Vector4f(depth/2, width/2, height, 0.0));

  Filter<geometry_msgs::Twist> filter(node,
                                      "input_velocity",
                                      "cmd_vel",
                                      10);

  LightCurtain curtain(
      node,
      base_frame,
      boost::bind(&BoxRobotBody::isNearBody, &robot, _1),
      boost::bind(&Filter<geometry_msgs::Twist>::set_danger, &filter, _1),
      ros::Duration(keep_duration));
  curtain.init();
  ros::spin();

  return 0;
}
