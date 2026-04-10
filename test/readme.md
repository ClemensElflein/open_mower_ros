# OpenMower ROS - Simulation Testing on WSL2

This guide documents how to set up a local simulation environment for testing OpenMower on Windows using WSL2 with Ubuntu 20.04 and ROS Noetic.

## Prerequisites

- Windows 11 Pro with WSL2 enabled
- Ubuntu 20.04 WSL distro (ROS Noetic requires Focal)

## 1. Install Ubuntu 20.04 WSL Distro

If you already have a different Ubuntu version, install 20.04 alongside it:

```bash
wsl --install Ubuntu-20.04
```

All subsequent commands run inside this distro:

```bash
wsl -d Ubuntu-20.04
```

## 2. Install ROS Noetic

```bash
# Add ROS apt repository
apt-get update && apt-get install -y curl gnupg lsb-release
echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
apt-get update

# Install ROS Noetic desktop
apt-get install -y ros-noetic-desktop

# Source ROS in your shell
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source /opt/ros/noetic/setup.bash
```

## 3. Install Build Dependencies

### ROS packages

```bash
apt-get install -y \
  ros-noetic-catkin \
  ros-noetic-mbf-costmap-nav \
  ros-noetic-mbf-costmap-core \
  ros-noetic-mbf-msgs \
  ros-noetic-costmap-2d \
  ros-noetic-nav-core \
  ros-noetic-global-planner \
  ros-noetic-pluginlib \
  ros-noetic-joy \
  ros-noetic-teleop-twist-joy \
  ros-noetic-twist-mux \
  ros-noetic-rosbridge-server \
  ros-noetic-robot-localization \
  ros-noetic-ublox \
  ros-noetic-imu-tools \
  ros-noetic-imu-filter-madgwick \
  ros-noetic-grid-map \
  ros-noetic-grid-map-core \
  ros-noetic-grid-map-ros \
  ros-noetic-grid-map-filters \
  ros-noetic-grid-map-cv \
  ros-noetic-pcl-conversions \
  ros-noetic-dynamic-reconfigure \
  ros-noetic-rosbag \
  ros-noetic-rostest \
  ros-noetic-serial \
  ros-noetic-nmea-msgs \
  ros-noetic-tf2-eigen \
  ros-noetic-mavros-msgs \
  ros-noetic-mobile-robot-simulator \
  ros-noetic-rosbag-snapshot \
  ros-noetic-rosbag-snapshot-msgs \
  python3-catkin-tools \
  python3-rosdep \
  python3-rosinstall \
  python3-rosinstall-generator \
  python3-wstool
```

### System libraries

```bash
apt-get install -y \
  build-essential \
  cmake \
  git \
  libcrypto++-dev \
  libasio-dev \
  libwebsocketpp-dev \
  libspdlog-dev \
  libeigen3-dev \
  libserial-dev \
  libssl-dev
```

### paho-mqtt-cpp (build from source)

This library is not in the Ubuntu 20.04 repos and must be built manually:

```bash
# paho.mqtt.c
cd /tmp
git clone --depth 1 https://github.com/eclipse/paho.mqtt.c.git
cd paho.mqtt.c
cmake -Bbuild -DPAHO_WITH_SSL=ON -DPAHO_ENABLE_TESTING=OFF -DCMAKE_INSTALL_PREFIX=/usr
cmake --build build -j$(nproc)
cmake --install build

# paho.mqtt.cpp
cd /tmp
git clone --depth 1 https://github.com/eclipse/paho.mqtt.cpp.git
cd paho.mqtt.cpp
cmake -Bbuild -DPAHO_WITH_SSL=ON -DCMAKE_INSTALL_PREFIX=/usr
cmake --build build -j$(nproc)
cmake --install build
ldconfig
```

**Important:** After installing, you must patch the CMake config to work with catkin's CMake version:

```bash
# Fix ALIAS target issue in PahoMqttCpp CMake config
python3 -c "
path = '/usr/lib/x86_64-linux-gnu/cmake/PahoMqttCpp/PahoMqttCppConfig.cmake'
with open(path) as f:
    content = f.read()
content = content.replace(
    'add_library(PahoMqttCpp::paho-mqttpp3 ALIAS PahoMqttCpp::paho-mqttpp3-shared)',
    'set_target_properties(PahoMqttCpp::paho-mqttpp3-shared PROPERTIES IMPORTED_GLOBAL TRUE)\n        add_library(PahoMqttCpp::paho-mqttpp3 ALIAS PahoMqttCpp::paho-mqttpp3-shared)'
).replace(
    'add_library(PahoMqttCpp::paho-mqttpp3 ALIAS PahoMqttCpp::paho-mqttpp3-static)',
    'set_target_properties(PahoMqttCpp::paho-mqttpp3-static PROPERTIES IMPORTED_GLOBAL TRUE)\n        add_library(PahoMqttCpp::paho-mqttpp3 ALIAS PahoMqttCpp::paho-mqttpp3-static)'
)
with open(path, 'w') as f:
    f.write(content)
print('Patched')
"
```

### Initialize rosdep

```bash
rosdep init
rosdep update
```

## 4. Clone and Build

Clone the fork with all submodules:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/alexvaut/open_mower_ros.git .
git submodule update --init --recursive
```

> **Note:** Build inside the WSL filesystem (`~/catkin_ws`), not on `/mnt/c/...`. Cross-filesystem builds are much slower.

Install any remaining rosdep dependencies:

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

Build:

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
catkin_make -j$(nproc)
```

The build produces ~21 packages. Expected warnings:
- `xbot_monitoring` depending on metapackage `grid_map` (harmless)
- Deprecated `tf2::Quaternion` constructor in slic3r_coverage_planner (harmless)

## 5. Run the Simulation

### Interactive simulation (with RViz)

```bash
source ~/catkin_ws/devel/setup.bash

# Set required environment variables
export OM_DOCKING_DISTANCE=1.0
export OM_UNDOCK_DISTANCE=1.0
export OM_TOOL_WIDTH=0.14
export OM_ENABLE_MOWER=false
export OM_BATTERY_EMPTY_VOLTAGE=20.0
export OM_OUTLINE_COUNT=4
export OM_OUTLINE_OFFSET=0.05
export OM_BATTERY_FULL_VOLTAGE=28.0
export OM_MOWING_MOTOR_TEMP_HIGH=80.0
export OM_MOWING_MOTOR_TEMP_LOW=40.0
export OM_MQTT_ENABLE=False
export OM_MOWER_GAMEPAD=""

roslaunch open_mower sim_mower_logic.launch
```

> RViz GUI works on Windows 11 via WSLg. If RViz doesn't appear, set `OM_START_RVIZ=False`.

### Headless simulation test

```bash
source ~/catkin_ws/devel/setup.bash
rostest mower_logic sim_startup.test --text
```

## 6. Test Structure

### Existing test: `sim_startup.test`

Location: `src/mower_logic/test/sim_startup.test`

This test launches the full simulation stack headlessly and verifies:

| Test | What it checks |
|------|---------------|
| `test_hardware_status_published` | `/mower/status` is published, battery voltage > 0 |
| `test_high_level_state_published` | `/mower_logic/current_state` is published within 20s |
| `test_initial_state_is_idle` | State machine starts in IDLE (state=1) |

**Nodes launched:**
- `mobile_robot_simulator` - simulated odometry and tf
- `mower_simulation` - simulated hardware (status, GPS, IMU, emergency, mower blade)
- `mower_map` - map storage service
- `slic3r_coverage_planner` - coverage path planner
- `move_base_flex` - navigation stack (MBF with FTC local planner)
- `mower_logic` - the main state machine under test
- `twist_mux` - velocity command arbitration

### Writing new tests

To add a new test:

1. Create a `.test` launch file in `src/mower_logic/test/` (roslaunch XML format)
2. Create a Python test script using `rostest` + `unittest`
3. Register it in `src/mower_logic/CMakeLists.txt`:

```cmake
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/sim_startup.test)
  add_rostest(test/your_new_test.test)  # add here
endif()
```

### Test ideas for stuck scenarios

Here are patterns for testing stuck/obstacle behavior:

**Simulate GPS degradation:**
```python
# Publish bad GPS accuracy to trigger GPS loss handling
from std_msgs.msg import Float64
pub = rospy.Publisher('/sensors/om_gps_accuracy/data', Float64, queue_size=1)
pub.publish(Float64(data=5.0))  # 5m accuracy = bad GPS
```

**Simulate stuck (robot not moving):**
```python
# Start mowing, then stop publishing odometry updates
# or publish the same pose repeatedly
# mower_logic should detect no progress and pause
```

**Simulate emergency:**
```python
from std_srvs.srv import SetBool
emergency = rospy.ServiceProxy('/mower_service/emergency', SetBool)
emergency(True)  # trigger emergency
# verify mower_logic enters emergency state
# then clear and verify recovery
emergency(False)
```

**Test teleop override while paused:**
```python
from geometry_msgs.msg import Twist
pub = rospy.Publisher('/override_vel', Twist, queue_size=1)
# Publish backward velocity while mower is paused
twist = Twist()
twist.linear.x = -0.5
pub.publish(twist)
```

## 7. Troubleshooting

### `$(find open_mower)` resolves to empty
Make sure you source the workspace: `source ~/catkin_ws/devel/setup.bash`

### `twist_mux` dies with "could not load parameter 'locks'"
Add `locks: []` to the twist_mux rosparam config.

### paho-mqtt-cpp "ALIAS target not globally visible"
Apply the patch described in step 3.

### mower_logic stuck at "Waiting for..." 
mower_logic waits for 11 services at startup. Make sure all required nodes are launched:
- `mower_simulation` provides: emergency, mow_enabled, set_gps_state, set_robot_pose
- `mower_map` provides: map CRUD services, docking point, nav points
- `move_base_flex` provides: navigation action server
- `slic3r_coverage_planner` provides: plan_path service
