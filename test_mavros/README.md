MAVROS test package
===================

This package consists hand-tests with FCU SITL environment.
I hope later we will do automatic tests too.


APM SITL
--------

All what you need described in [ardupilot wiki][sitl-wiki].


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


### Run simulation

```
./sim_vehicle.sh -v ArduPlane --out udp:localhost:15550 --map
roslaunch test_mavros launch/apm/apm_imu_test.launch
```


[sitl-wiki]: http://dev.ardupilot.com/wiki/simulation-2/sitl-simulator-software-in-the-loop/setting-up-sitl-on-linux/
