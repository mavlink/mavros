MAVROS test package
===================

This package consists hand-tests with FCU SITL environment.
I hope later we will do automatic tests too.

PX4 ROS SITL
------------
Follow the instructions presented on [PX4 ROS SITL Setup][px4-sitl-wiki].

To test the simulation environment all you have to do is launch the propper ROS launch file. Right now, the current available one is `iris_empty_world_offboard_ctl.launch`, which allows to test the offboard control routines of the Firmware, together with the MAVROS API.

### Available tests

#### Offboard position and velocity control
Note: acceleration control still not supported on PX4 Firmware side.

##### Tested in launch files

- `iris_empty_world_offboard_ctl.launch`

##### Description

Allows testing the offboard control routines of the PX4 firmware by issuing setpoint commands through MAVROS plugins. Current test implements code to send:

- position setpoints
- velocity setpoints

The tests are implemented by issuing some kind of shaped path. Current shapes are:

- square/rectangle
- circle
- eight
- ellipse (3D)

##### How to use

To test the different behaviors, edit `iris_empty_world_offboard_ctl.launch`. At the bottom of this file, you will find:

```xml
	<!-- SITL test base node launcher -->
    <arg name="mode" default="position" />    <!-- position ctl mode -->
    <arg name="shape" default="square" />    <!-- square shaped path -->
```

Just change the default value of them and issue `roslaunch test_mavros iris_empty_world_offboard_ctl.launch`

Or, you can just issue the roslaunch passing the parameter values on the command line, p.e. `roslaunch test_mavros iris_empty_world_offboard_ctl.launch mode:=position shape:=square`.

##### TODO

- Implement acceleration setpoint sending, when this is implemented on Firmware side
- Give possibility to users to define the amplitude of movement
- Implement a PID controller for velocity to avoid overshoots in the onboard controller



APM SITL
--------

All what you need described in [ardupilot wiki][apm-sitl-wiki].


### Preparation

```sh
# this is for zsh, but bash should be similar
function add-dir-to-path() {
    PATH+=":$1"
}

# get sources
cd ~/ros/src
wstool set imu_tools --git https://github.com/ccny-ros-pkg/imu_tools.git -v indigo
wstool update -j2
catkin build

cd ~/src/UAV
git clone https://github.com/diydrones/ardupilot.git
git checkout ArduPlane-3.3.0 -b ArduPlane-3.3.0
git clone https://github.com/tridge/jsbsim.git

# compile JSBSim
cd jsbsim
./autogen.sh --enable-libraries
make -j4

# also look ardupliot wiki

# prepare to sim_vehicle.sh
add-dir-to-path $PWD/src

cd ../ardupilot/Tools/autotest/
add-dir-to-path $PWD
```


### How to use

```
./sim_vehicle.sh -v ArduPlane --out udp:localhost:15550 --map
roslaunch test_mavros launch/apm/apm_imu_test.launch
```

### Screen capture for that test (youtube video)

[![APM SITL test video](http://img.youtube.com/vi/mUIptiNbmS4/0.jpg)](http://www.youtube.com/watch?v=mUIptiNbmS4)


[apm-sitl-wiki]: http://dev.ardupilot.com/wiki/simulation-2/sitl-simulator-software-in-the-loop/setting-up-sitl-on-linux/
[px4-sitl-wiki]: https://pixhawk.org/dev/ros/sitl
