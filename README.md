# scout_ros
[![Licence](https://img.shields.io/badge/License-BSD--3-green.svg)](https://opensource.org/license/bsd-3-clause/)
[![ubuntu20](https://img.shields.io/badge/-UBUNTU_20.04-orange?style=flat-square&logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/focal/)
[![noetic](https://img.shields.io/badge/-NOETIC-blue?style=flat-square&logo=ros)](https://wiki.ros.org/noetic)

## Overview
ROS packages for Scout 2.0

[Specification](https://roas.co.kr/scout-2-0/)<br>
[Manual](https://docs.roas.co.kr/scout.html)

## Installation

### Install packages
```
cd ~/catkin_ws/src/
git clone https://github.com/roasinc/scout_ros.git
```

### Build
```
cd ~/catkn_ws/
rosdep install --from-paths src --ignore-src -y
catkin_make
```

## Usage
```
sudo ip link set can0 up type can bitrate 500000
roslaunch scout_base base.launch
```
