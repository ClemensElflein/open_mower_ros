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


double minX,minY,minZ,maxX,maxY,maxZ;

double filteredX, filteredY, filteredZ;
uint valueCount = 0;

double filterFactor = 0.9;

void magReceived(const sensor_msgs::MagneticField::ConstPtr &msg) {
    if(valueCount == 0) {
        filteredX = msg->magnetic_field.x;
        filteredY = msg->magnetic_field.y;
        filteredZ = msg->magnetic_field.z;
        valueCount++;
        return;
    }

    valueCount++;


    filteredX = filterFactor * filteredX + (1.0-filterFactor) * msg->magnetic_field.x;
    filteredY = filterFactor * filteredY+ (1.0-filterFactor) * msg->magnetic_field.y;
    filteredZ = filterFactor * filteredZ + (1.0-filterFactor) * msg->magnetic_field.z;

    if(valueCount < 100) {
        return;
    }

    minX = std::min(minX, filteredX);
    minY = std::min(minY, filteredY);
    minZ = std::min(minZ, filteredZ);
    maxX = std::max(maxX, filteredX);
    maxY = std::max(maxY, filteredY);
    maxZ = std::max(maxZ, filteredZ);


    ROS_INFO_STREAM_THROTTLE(1,
                             "\n" <<
                             "min_x = " << minX << "; max_x = " << maxX<< "; x_bias = " << (minX+maxX)/2.0 << "\n" <<
                             "min_y = " << minY << "; max_y = " << maxY<< "; y_bias = " << (minY+maxY)/2.0 << "\n"<<
                             "min_z = " << minZ << "; max_z = " << maxZ<< "; z_bias = " << (minZ+maxZ)/2.0 << "\n"
    );
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "mag_calibration");

    minX = minY = minZ=INFINITY;
    maxX = maxY = maxZ = -INFINITY;

    std::cout << "Rotate the mower around all axes and press enter once you are done.";
    sleep(2);

    ros::NodeHandle n;
    ros::Subscriber mag_sub = n.subscribe("imu/mag", 100, magReceived);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    while(ros::ok()) {
        std::string line;
        std::getline(std::cin, line);
        break;
    }

    std::cout << "Done, use the following lines in your config:\n" <<
                            "export OM_MAG_BIAS_X=" << (minX+maxX)/2.0 << "\n" <<
                            "export OM_MAG_BIAS_Y=" << (minY+maxY)/2.0 << "\n" <<
                            "export OM_MAG_BIAS_Z=" << (minZ+maxZ)/2.0 << "\n";



    return 0;
}
