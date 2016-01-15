#include <boost/shared_ptr.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "vesc_ackermann/vesc_to_odom.h"

namespace vesc_ackermann
{

class VescToOdomNodelet: public nodelet::Nodelet
{
public:

  VescToOdomNodelet() {}

private:

  virtual void onInit(void);

  boost::shared_ptr<VescToOdom> vesc_to_odom_;

}; // class VescToOdomNodelet

void VescToOdomNodelet::onInit()
{
  NODELET_DEBUG("Initializing RACECAR VESC odometry estimator nodelet");
  vesc_to_odom_.reset(new VescToOdom(getNodeHandle(), getPrivateNodeHandle()));
}

} // namespace vesc_ackermann

PLUGINLIB_EXPORT_CLASS(vesc_ackermann::VescToOdomNodelet, nodelet::Nodelet);
