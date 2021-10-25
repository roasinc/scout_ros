Scout 2.0 ROS Packages
=======================

Overview
---------
ROS packages for Scout 2.0

[Scout 2.0 Tutorial](https://docs.roas.co.kr/scout.html)

Installation
------------

```
cd ~/catkin_ws/src/
git clone https://github.com/roasinc/scout_ros.git
cd ~/catkin_ws/src/scout_ros/scout_base/lib/
sudo dpkg -i ros-melodic-scout-lib-* (your PC architecture)

cd ~/catkn_ws/
rosdep install --from-paths src --ignore-src -y
catkin_make
```

Start
-----

```
sudo chmod 666 /dev/ttyUSB0
roslaunch scout_mini_base base.launch
```

Upstart
-------

```
rosrun scout_lib install_upstart -r scout
sudo reboot
```
