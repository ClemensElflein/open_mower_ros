## I am available for hire
Hello! With a background in software engineering, embedded programming, hardware design, and robotics, I'm on the lookout for new challenges.
If you're in search of someone with my skills, let's team up and create something amazing! https://x-tech.online/

# ROS Workspace

[![Build](https://github.com/ClemensElflein/open_mower_ros/actions/workflows/build-image.yaml/badge.svg)](https://github.com/ClemensElflein/open_mower_ros/actions/workflows/build-image.yaml)

This folder is the ROS workspace, which should be used to build the OpenMower ROS software.
This repository contains the ROS package for controlling the OpenMower.

There are references to other repositories (libraries) needed to build the software. This way, we can track the exact version of the packages used in each release to ensure package compatibility.
Currently, the following repositories are included:

- **slic3r_coverage_planner**: A coverage planner based on the Slic3r software for 3d printers. This is used to plan the mowing path.
- **teb_local_planner**: The local planner which allows the robot to avoid obstacles and follow the global path using kinematic constraints.
- **xesc_ros**: The ROS interface for the xESC motor controllers.

## Container images: Default vs Legacy

If your robot runs the latest OpenMower OS (v2): use the images without prefix or suffix (e.g. `latest`, `v1.2.3`).
These images only contain the OpenMower ROS stack and expect the OS to provide web and MQTT services (for example via your system’s compose setup).

If your robot runs an old version of OpenMower OS v1 (Legacy): use the legacy image.
The OS doesn't provide web and MQTT services, so the image contains nginx and mosquitto to provide these services inside the container.
The Docker images have a `-legacy` suffix or `releases-` prefix: (e.g. `releases-edge`, `v1.2.3-legacy`).

## Getting started

### Running on your machine

OpenMower requires ROS Noetic. ([installation instruction](http://wiki.ros.org/noetic/Installation)) There is no distributed release package yet, for development and test purpose it's best to build the workspace on your own.

#### Fetch Dependencies

Before building, you need to fetch this project's dependencies. The best way to do this is by using rosdep:

```bash
sudo apt install python3-rosdep
sudo rosdep init
```

Run in the repository's root:

```bash

rosdep update
git submodule update --init --recursive
rosdep install --from-paths src --ignore-src --default-yes
```

#### Build workspace

Just build as any other ROS workspace: `catkin_make`
Once it's done, another step is to source workspace env vars:

```bash
source devel/setup.bash
```

#### Launch OpenMower

OpenMower ROS package is distributed with [roslaunch](http://wiki.ros.org/roslaunch) launch files.
There are few in: `src/open_mower/open_mower/launch`, however the `open_mower.launch` runs everything needed to mow.

```bash
roslaunch open_mower open_mower.launch
```

Before you launch `open_mower` package, env vars with configuration have to be set.

```bash
cp src/open_mower/config/mower_config.sh.example mower_config.sh
source mower_config.sh # it's expected to adjust the file
```

### Running in a container

TBD (no automated image build yet)

## Contribution

### How to Build Using CLion IDE

First, launch CLion in a sourced environment. For this I use the following bash file:

```bash
#!/bin/zsh

source <your_absolute_path_to_repository>/devel/setup.zsh

# You can find this path in the Jetbrains Toolbox
nohup <your_absolute_path_to_clion>/clion.sh >/dev/null 2>&1 &
```


Then, open the `src` directory. CLion will prompt with the following screen:

![CLion CMake Settings](./img/clion_cmake_settings.png)

Copy the settings for **Build directory** and **CMake options**. Everything else can stay the same. This is all you need!


# Notes / ToDos

- For local navigation, I have tried to use the teb_local_planner. Unfortunately, it seems that (at least for me) the noetic version is VERY broken. Therefore I added the current melodic dev version as git submodule to this repo. It seems to work fine with ROS noetic and this setup here.
- If the map has no docking point set, planning crashes as soon as we try to approach the docking point. TODO: check, before even starting to mow.

# License

This work is licensed under the [GNU General Public License version 3](https://www.gnu.org/licenses/gpl-3.0.html). See the [LICENSE](LICENSE) file for details.
