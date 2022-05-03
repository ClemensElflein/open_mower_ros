// Created by Clemens Elflein on 2/21/22.
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
#include <dynamic_reconfigure/server.h>
#include "IdleBehavior.h"

extern void stop();
extern void setEmergencyMode(bool emergency);

extern mower_msgs::Status last_status;
extern bool mowingPaused;
extern mower_logic::MowerLogicConfig last_config;
extern dynamic_reconfigure::Server<mower_logic::MowerLogicConfig> *reconfigServer;

IdleBehavior IdleBehavior::INSTANCE;

std::string IdleBehavior::state_name() {
    return "IDLE";
}

Behavior *IdleBehavior::execute() {


    ros::Rate r(1);
    while (ros::ok()) {
        stop();

        if (last_config.manual_start_mowing ||
            (last_status.v_battery > config.battery_full_voltage && last_status.mow_esc_status.temperature_motor < 45.0 &&
             !last_config.manual_pause_mowing)) {
            mowingPaused = false;
            return &UndockingBehavior::INSTANCE;
        }

        r.sleep();
    }

    return nullptr;
}

void IdleBehavior::enter() {
    // disable it again so that we don't get stuck in a loop and drain the battery
    if (last_config.manual_start_mowing) {
        last_config.manual_start_mowing = false;
        reconfigServer->updateConfig(last_config);
    }
}

void IdleBehavior::exit() {
    // disable emergency during undocking
    setEmergencyMode(false);
    // disable it again so that we don't get stuck in a loop and drain the battery
    if (last_config.manual_start_mowing) {
        last_config.manual_start_mowing = false;
        reconfigServer->updateConfig(last_config);
    }
}

void IdleBehavior::reset() {

}

bool IdleBehavior::needs_gps() {
    return false;
}

bool IdleBehavior::mower_enabled() {
    return false;
}
