# OpenMower ROS

![OpenMower the DIY smart robot mower](./img/open_mower_header.jpg)[![License: CC BY-NC-SA 4.0](https://img.shields.io/badge/License-CC%20BY--NC--SA%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc-sa/4.0/)

This repository contains the ROS package for controlling the OpenMower.

# Installation

TODO: Add installation instructions here

Hint: To install dependencies, run the following:
```bash
rosdep install --ignore-src open_mower
```
This will ignore local packages (since they are not yet available in the ROS index).

# Notes / ToDos
- For local navigation, I have tried to use the teb_local_planner. Unfortunately, it seems that (at least for me) the noetic version is VERY broken. Therefore I added the current melodic dev version as git submodule to this repo. It seems to work fine with ROS noetic and this setup here.
- If the map has no docking point set, planning crashes as soon as we try to approach the docking point. TODO: check, before even starting to mow.
# License

<a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-nc-sa/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/">Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License</a>.

Feel free to use the design in your private/educational projects, but don't try to sell the design or products based on it without getting my consent first. The idea here is to share knowledge, not to enable others to simply sell my work. Thank you for understanding.