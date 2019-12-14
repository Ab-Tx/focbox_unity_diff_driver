# FOCBOX Unity driver

simple api to comunicate with FOCBOX Unity motor controllers

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

What things you need to install the software and how to install them.

packages may be under different names, use the package manager of you choosing.

```
ros-melodic
ros-melodic-ros-control
ros-melodic-ros-controllers
ros-melodic-velocity-controllers
```


### Installing

A step by step series of examples that tell you how to get a development env running

setup your catkin workspace if it isn't already

```
https://wiki.ros.org/catkin/Tutorials/create_a_workspace
```

clone this repository to your catkin workspace

```
git clone https://github.com/gimbas/focbox_unity_diff_driver
```

compile the node
```
catkin_make
```

if you wish you can deploy/install
```
catkin_make --install
```

now you can launch node

make sure you have the workspace overlayed on terminal instance
```
bash your_catkin_workspace/devel/setup.bash
```

and ros is runing
```
roscore
```

launch
```
roslaunch focbox_unity_diff_driver/ focbox_unity_diff_driver_node.launch
```

## Test

you can send a velocity command to make sure it is working
```
rostopic pub /cmd_vel geometry_msgs/Twist '[$linear velocity (m/s)$, 0, 0]' '[0, 0, $angular velocity (rad/s)$]'
```

## Built With

* catkin
* roscpp
* [C++](//) - C++ language

## API & documentation

API documentation is available in [API.md](API.md)

## Contributing

Currently not looking for external contributers.

Please read [CONTRIBUTING.md](//) for details on our code of conduct, and the process for submitting pull requests to us.

## Authors

* **Rafael Silva** - *complete rework* - [gimbas](https://github.com/gimbas)

See also the list of [contributors](https://github.com/gimbas/focbox_unity_ros_driver/graphs/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* original vesc firmware - [vedderb](https://github.com/vedderb/bldc)
* focbox unity firmware - [enertionboards](https://github.com/EnertionBoards/bldc/tree/unity)
* mit racecar vesc driver - [mit-racecar/vesc](https://github.com/mit-racecar/vesc)
* ros_control & roscontrollers