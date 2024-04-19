// Created by Bodo Pfelzer on 28/10/23.
// Copyright (c) 2023 Bodo Pfelzer. All rights reserved.
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
#ifndef SRC_PERIMETER_DOCKING_H
#define SRC_PERIMETER_DOCKING_H

#include "Behavior.h"

/*
#include <actionlib/client/simple_action_client.h>
#include <mbf_msgs/ExePathAction.h>
#include <mbf_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include "ros/ros.h"
#include <tf2/LinearMath/Transform.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "mower_msgs/Status.h"
#include <mower_map/GetDockingPointSrv.h>
*/

class PerimeterBase : public Behavior {
public:
  void enter() override;
  void exit() override;
  void reset() override;
  bool needs_gps() override;
  bool mower_enabled() override;
  void command_home() override;
  void command_start() override;
  void command_s1() override;
  void command_s2() override;
  bool redirect_joystick() override;
  uint8_t get_sub_state() override;
  uint8_t get_state() override;
  void handle_action(std::string action) override;
protected:
  int setupConnections();
};

class PerimeterFollowBehavior : public PerimeterBase {
public:
  Behavior *execute() override;
protected:
  /**
   * @brief distance travelled.
   */
  double travelled;
  /**
   * @brief We arrived and should continue with the returned Behavior.
   */
  virtual Behavior* arrived()=0;
};

class PerimeterDockingBehavior : public PerimeterFollowBehavior {
private:
  int chargeSeen;
public:
  static PerimeterDockingBehavior INSTANCE;
  std::string state_name() override;
protected:
  Behavior* arrived() override;
};

class PerimeterSearchBehavior : public PerimeterBase {
public:
  static PerimeterSearchBehavior INSTANCE;
  /**
   * Is usage configured?
   */
  static int configured(const mower_logic::MowerLogicConfig &config);


  std::string state_name() override;
  Behavior *execute() override;
};

class PerimeterUndockingBehavior : public PerimeterBase {
public:
  static PerimeterUndockingBehavior INSTANCE;
  /**
   * Is usage configured?
   */
  static int configured(const mower_logic::MowerLogicConfig &config);

  std::string state_name() override;
  Behavior *execute() override;
};

class PerimeterMoveToGpsBehavior : public PerimeterFollowBehavior {
public:
  static PerimeterMoveToGpsBehavior INSTANCE;
  std::string state_name() override;
  void enter() override;
protected:
  Behavior* arrived() override;
};

#endif //SRC_PERIMETER_DOCKING_H

