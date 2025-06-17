//
// Created by clemens on 03.07.22.
//

#ifndef SRC_XESC_YFR4_DRIVER_H
#define SRC_XESC_YFR4_DRIVER_H

#include <ros/ros.h>
#include <xesc_interface/xesc_interface.h>
#include "xesc_yfr4_interface.h"

namespace xesc_yfr4_driver  {
    class XescYFR4Driver: public xesc_interface::XescInterface {
    public:
        XescYFR4Driver(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
        void getStatus(xesc_msgs::XescStateStamped &state) override;

        void getStatusBlocking(xesc_msgs::XescStateStamped &state) override;

        void setDutyCycle(float duty_cycle) override;

        void stop() override;

    private:
        void error_func(const std::string &s);
        xesc_yfr4_driver::XescYFR4StatusStruct status{};
        xesc_yfr4_driver::XescYFR4Interface* xesc_interface = nullptr;
    };
}

#endif  // SRC_XESC_YFR4_DRIVER_H
