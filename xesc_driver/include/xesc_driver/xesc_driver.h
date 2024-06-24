//
// Created by clemens on 03.07.22.
//

#ifndef SRC_XESC_DRIVER_H
#define SRC_XESC_DRIVER_H

#include <ros/ros.h>
#include <xesc_interface/xesc_interface.h>
#include "xesc_2040_driver/xesc_2040_driver.h"
#include "vesc_driver/vesc_driver.h"
#include "xesc_yfr4_driver/xesc_yfr4_driver.h"


namespace xesc_driver  {
    class XescDriver: public xesc_interface::XescInterface {
    public:
        XescDriver(ros::NodeHandle &nh, ros::NodeHandle &private_nh);
        ~XescDriver();

        void getStatus(xesc_msgs::XescStateStamped &state) override;

        void getStatusBlocking(xesc_msgs::XescStateStamped &state) override;

        void setDutyCycle(float duty_cycle) override;

        void stop() override;

    private:
        xesc_interface::XescInterface *xesc_driver = nullptr;
    };
}

#endif //SRC_XESC_DRIVER_H
