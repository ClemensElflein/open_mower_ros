#include <boost/shared_ptr.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "vesc_ackermann/ackermann_to_vesc.h"

namespace vesc_ackermann
{

class AckermannToVescNodelet: public nodelet::Nodelet
{
public:

  AckermannToVescNodelet() {}

private:

  virtual void onInit(void);

  boost::shared_ptr<AckermannToVesc> ackermann_to_vesc_;

}; // class AckermannToVescNodelet

void AckermannToVescNodelet::onInit()
{
  NODELET_DEBUG("Initializing ackermann to VESC nodelet");
  ackermann_to_vesc_.reset(new AckermannToVesc(getNodeHandle(), getPrivateNodeHandle()));
}

} // namespace vesc_ackermann

PLUGINLIB_EXPORT_CLASS(vesc_ackermann::AckermannToVescNodelet, nodelet::Nodelet);
