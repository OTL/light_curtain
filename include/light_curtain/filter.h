//
// @author Takashi Ogura <t.ogura@gmail.com>
//
// new BSD license
//

#ifndef LIGHT_CURTAIN_FILTER_H
#define LIGHT_CURTAIN_FILTER_H

#include <string>
#include <ros/ros.h>

namespace light_curtain {

/// filtering a topic by set_danger
template <class T>
class Filter {
 public:
  Filter(ros::NodeHandle& node,
         const std::string& input_name,
         const std::string& output_name,
         const int32_t buffer_size)
      : is_danger_(false)
      , node_(node){
    publisher_ = node_.advertise<T>(output_name, buffer_size);
    subscriber_ = node_.subscribe(input_name, buffer_size,
                                  &Filter<T>::callback, this);
  }

  void callback(const typename T::ConstPtr& msg) {
    if (!is_danger_) {
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

#endif  // LIGHT_CURTAIN_FILTER_H
