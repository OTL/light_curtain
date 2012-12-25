#include <light_curtain/velocity_curtain_nodelet.h>
#include <nodelet/loader.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velocity_curtain");
  nodelet::Loader manager(true);
  nodelet::M_string remappings;
  nodelet::V_string my_argv;

  manager.load(ros::this_node::getName(),
               "light_curtain/VelocityCurtainNodelet",
               remappings,
               my_argv);

  ros::spin();
  return 0;
}
