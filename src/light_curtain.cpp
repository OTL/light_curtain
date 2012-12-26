//
// @author Takashi Ogura <t.ogura@gmail.com>
//
// new BSD license
//

#include <light_curtain/light_curtain.h>

namespace light_curtain {

LightCurtain::LightCurtain(const JudgeDanger& judge_callback,
                           const InformFunction& inform_callback,
                           double keep_duration)
    : judge_callback_(judge_callback)
    , inform_callback_(inform_callback)
    , keep_duration_(keep_duration)
    , is_danger_(false)
{

}

bool LightCurtain::getDangerState() const {
  return is_danger_;
}

void LightCurtain::init()
{
  last_danger_stamp_ = ros::Time::now();
}

void LightCurtain::setDangerState(bool is_danger) {
  if (is_danger_ != is_danger) {
    inform_callback_(is_danger);
  }
  is_danger_ = is_danger;
}

void LightCurtain::updatePointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
  ros::Time current_stamp = ros::Time::now();
  if (judge_callback_(cloud)) {
    // danger
    last_danger_stamp_ = current_stamp;
    setDangerState(true);
  } else if (current_stamp - last_danger_stamp_ > ros::Duration(keep_duration_)) {
    setDangerState(false);
  }
}

}
