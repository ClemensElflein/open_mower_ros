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

void testEmergency() {
    std::cout << "Emergency Switch Test." << std::endl;
    std::cout << "Enable and disable all emergency switches for the test to complete." << std::endl;

    uint8_t emergencies_seen_low = 0;
    uint8_t emergencies_seen_high = 0;

    while(emergencies_seen_high != 0b1111 && emergencies_seen_low != 0b1111) {
        auto status_ptr = ros::topic::waitForMessage<mower_msgs::Status>("mower/status", ros::Duration(1, 0));
        if(!status_ptr) {
            std::cout << "WARNING: NO MOWER STATUS RECEIVED" << std::endl;
            continue;
        }

        emergencies_seen_low |= ~(status_ptr->emergency>>1);
        emergencies_seen_high |= (status_ptr->emergency>>1);

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
        std::cout << state.str();
    }

    std::cout << "SUCCESS!" << std::endl;
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "hardware_test");
    ros::NodeHandle n;

    testEmergency();
    return 0;
}
