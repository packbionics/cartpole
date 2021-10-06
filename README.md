# cartpole
Cartpole swingup and balancing on a physical robot

# Project Organization

```
cartpole - Top level git repository
  |
  |-cartpole - Support meta package
  |
  |-cartpole_control - Teleoperation & control systems
  |
  |-cartpole_description - Model, simulation & visualization
  |
  |-cartpole_interfaces - Messages & services
```

# Installation

## Prerequisites

### [ROS2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)  
### [Gazebo](http://gazebosim.org/tutorials?tut=install_ubuntu)

## Install
1. Clone repository into `~/dev_ws/src/cartpole`
2. Build packages: `cd ~/dev_ws && colcon build`
3. In a new terminal: `cd ~/dev_ws && . install/setup.bash`



