/*********************************************************************
 * Copyright (c) 2019, SoftBank Corp.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************/

#include "vesc_hw_interface/vesc_hw_interface.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vesc_hw_interface_node");

  ros::NodeHandle nh, nh_private("~");
  vesc_hw_interface::VescHwInterface vesc_hw_interface;
  vesc_hw_interface.init(nh, nh_private);

  controller_manager::ControllerManager controller_manager(&vesc_hw_interface, nh);

  ros::Rate loop_rate(1.0 / vesc_hw_interface.getPeriod().toSec());
  ros::AsyncSpinner spinner(1);

  spinner.start();

  while (ros::ok())
  {
    // sends commands
    vesc_hw_interface.write();

    // updates the hardware interface control
    controller_manager.update(vesc_hw_interface.getTime(), vesc_hw_interface.getPeriod());

    // gets current states
    vesc_hw_interface.read();

    // sleeps
    loop_rate.sleep();
  }

  spinner.stop();

  return 0;
}
