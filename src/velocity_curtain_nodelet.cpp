//
// @author Takashi Ogura <t.ogura@gmail.com>
//
// new BSD license
//

#include <light_curtain/velocity_curtain_nodelet.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(light_curtain,
                        VelocityCurtainNodelet,
                        light_curtain::VelocityCurtainNodelet,
                        nodelet::Nodelet)

namespace light_curtain {

/// Default constructor
VelocityCurtainNodelet::VelocityCurtainNodelet() {
}

void VelocityCurtainNodelet::onInit() {
  ros::NodeHandle node(getNodeHandle());
  ros::NodeHandle private_node(getPrivateNodeHandle());

  double width(0.5);
  private_node.param("robot_width", width, 0.5);
  double depth(0.5);
  private_node.param("robot_depth", depth, 0.5);
  double height(1.5);
  private_node.param("robot_height", height, 1.5);
  double keep_duration(1.0);
  private_node.param("keep_duration", keep_duration, 1.0);

  std::string base_frame("base_link");
  private_node.param<std::string>("base_frame_id", base_frame, "base_link");

  using Eigen::Vector4f;

  robot_ = boost::shared_ptr<BoxRobotBody>(
      new BoxRobotBody(Vector4f(-depth/2, -width/2, 0.0, 0.0),
                       Vector4f(depth/2, width/2, height, 0.0)));

  filter_ = boost::shared_ptr<ForwardVelocityFilter>(
      new ForwardVelocityFilter(node,
                                "input_velocity",
                                "output_velocity",
                                10));
  curtain_ = boost::shared_ptr<LightCurtain>(
      new LightCurtain(
          boost::bind(&BoxRobotBody::isNearBody, robot_, _1),
          boost::bind(&ForwardVelocityFilter::set_danger, filter_, _1),
          ros::Duration(keep_duration)));

  pointcloud_updater_ = boost::shared_ptr<PointCloudROS>(
      new PointCloudROS(
          node,
          "/curtain/points",
          base_frame,
          boost::bind(&LightCurtain::updatePointCloud, curtain_, _1)));

  laser_updater_ = boost::shared_ptr<LaserROS>(
      new LaserROS(
          node,
          "/curtain/scan",
          base_frame,
          boost::bind(&LightCurtain::updatePointCloud, curtain_, _1)));

  pointcloud_updater_->init();
  laser_updater_->init();
  curtain_->init();
}

/// Default destructor
VelocityCurtainNodelet::~VelocityCurtainNodelet() {
}


}  // namespace light_curtain
