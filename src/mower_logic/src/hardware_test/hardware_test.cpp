// Created by Clemens Elflein on 3/28/22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//
// This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
//
// Feel free to use the design in your private/educational projects, but don't try to sell the design or products based on it without getting my consent first.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//

#include "ros/ros.h"

#include <sensor_msgs/MagneticField.h>
#include "mower_msgs/Status.h"
#include "iostream"
#include "sensor_msgs/Imu.h"
#include <serial/serial.h>

mower_msgs::Status status_msg;
sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField mag_msg;

void status_cb(const mower_msgs::Status::ConstPtr &msg) {
    std::cout << "c" << std::endl;
    status_msg = *msg;
}

void imu_cb(const sensor_msgs::Imu::ConstPtr &msg) {
    imu_msg = *msg;
    std::cout << "a" << std::endl;
}
void mag_cb(const sensor_msgs::MagneticField::ConstPtr &msg) {
    mag_msg = *msg;
    std::cout << "b" << std::endl;
}



bool testEmergency() {
    std::cout << "Emergency Switch Test." << std::endl;
    std::cout << "Enable and disable all emergency switches for the test to complete." << std::endl;
    std::cout << "Press ENTER to start." << std::endl;
    std::cin.get();

    uint8_t emergencies_seen_low = 0;
    uint8_t emergencies_seen_high = 0;

    while(emergencies_seen_high != 0b1111 || emergencies_seen_low != 0b1111) {
        auto status_ptr = ros::topic::waitForMessage<mower_msgs::Status>("mower/status", ros::Duration(1, 0));
        if(!status_ptr) {
            std::cout << "WARNING: NO MOWER STATUS RECEIVED" << std::endl;
            continue;
        }

        emergencies_seen_low |= (~(status_ptr->emergency>>1)) & 0b1111;
        emergencies_seen_high |= ((status_ptr->emergency>>1)) & 0b1111;

        std::stringstream state;
        state << "LOW: [";
        if(emergencies_seen_low & 0b1) {
            state << "X,";
        } else {
            state << " ,";
        }
        if(emergencies_seen_low & 0b10) {
            state << "X,";
        } else {
            state << " ,";
        }
        if(emergencies_seen_low & 0b100) {
            state << "X,";
        } else {
            state << " ,";
        }
        if(emergencies_seen_low & 0b1000) {
            state << "X]";
        } else {
            state << " ]";
        }
        state << " HIGH: [";
        if(emergencies_seen_high & 0b1) {
            state << "X,";
        } else {
            state << " ,";
        }
        if(emergencies_seen_high & 0b10) {
            state << "X,";
        } else {
            state << " ,";
        }
        if(emergencies_seen_high & 0b100) {
            state << "X,";
        } else {
            state << " ,";
        }
        if(emergencies_seen_high & 0b1000) {
            state << "X]";
        } else {
            state << " ]";
        }
        std::cout << state.str() << std::endl;
    }

    std::cout << "SUCCESS!" << std::endl;
    return true;
}

bool testIMU() {
    std::cout << "IMU Test." << std::endl;
    std::cout << "Move the board and make sure all values are changing (and do make sense)" << std::endl;
    std::cout << "Press ENTER to start." << std::endl;
    std::cin.get();

    ros::Rate refresh(10);

    while(1) {

        std::stringstream state;

        state << "Accel: " << imu_msg.linear_acceleration.x << ", " << imu_msg.linear_acceleration.y << ", " << imu_msg.linear_acceleration.z << ", " << std::endl;
        state << "Gyro: " << imu_msg.angular_velocity.x << ", " << imu_msg.angular_velocity.y << ", " << imu_msg.angular_velocity.z << ", " << std::endl;
        state << "Mag: " << mag_msg.magnetic_field.x << ", " << mag_msg.magnetic_field.y << ", " << mag_msg.magnetic_field.z << ", " << std::endl;

        std::cout << state.str();

        refresh.sleep();

        std::cout << "\x1B[2J\x1B[H";


    }

    std::cout << "SUCCESS!" << std::endl;
    return true;
}

bool testGPSSerial(std::string portName) {
    serial::Serial serial_port;
    try {
        serial_port.setPort(portName);
        serial_port.setBaudrate(115200);
        auto to = serial::Timeout::simpleTimeout(100);
        serial_port.setTimeout(to);
        serial_port.open();
    } catch (std::exception &e) {
        ROS_ERROR_STREAM("Error during serial connect with port: " << serial_port.getPort());
        return false;
    }

    try {
        serial_port.flushOutput();
        serial_port.flushInput();
    } catch (std::exception &e) {
        ROS_ERROR_STREAM("Error reading serial_port. Closing Connection.");
        return false;
    }

    int fails = 0;
    bool success = false;
    while(1) {
        try {
            serial_port.write("This is a test string!\n");
            std::string read_back = serial_port.readline();
            ROS_INFO_STREAM("Got from serial: " << read_back);
            if (read_back != "This is a test string!\n") {
                ROS_ERROR_STREAM("Error: read string is not the same as written string. Bridge TX to RX and try again");
                sleep(1);
                fails++;
                if(fails >= 10) {
                    break;
                }
            } else {
                success = true;
                break;
            }
        } catch (std::exception &e) {
            ROS_ERROR_STREAM("Error reading serial_port. Closing Connection.");
            return false;
        }
    }
    if(success) {
        std::cout << "SUCCESS: RX was TX on GPS serial port" << std::endl;
    }
    return success;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "hardware_test");
    ros::NodeHandle n;
//
//    ros::AsyncSpinner s(1);
//    s.start();

//
//    ros::Subscriber mag_sub = n.subscribe("imu/mag", 1000, mag_cb);
//    ros::Subscriber imu_sub = n.subscribe("imu/data_raw", 1000, imu_cb);
//    ros::Subscriber status_sub = n.subscribe("mower/status", 1000, status_cb);
//
//    ros::spin();
//
//    if(!testIMU()) {
//        std::cout << "ERROR: IMU TEST FAILED!";
//        delete(n);
//        return 1;
//    }

    if(!testGPSSerial("/dev/ttyAMA1")) {
        std::cout << "ERROR: GPS SERIAL TEST FAILED!";
        return 1;
    }

    if(!testEmergency()) {
        std::cout << "ERROR: EMERGENCY TEST FAILED!";
//        delete(n);
        return 1;
    }




//    delete(n);
    return 0;
}
