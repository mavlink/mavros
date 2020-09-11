^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_mavros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.4.0 (2020-09-11)
------------------

1.3.0 (2020-08-08)
------------------

1.2.0 (2020-05-22)
------------------

1.1.0 (2020-04-04)
------------------

1.0.0 (2020-01-01)
------------------

0.33.4 (2019-12-12)
-------------------

0.33.3 (2019-11-13)
-------------------

0.33.2 (2019-11-13)
-------------------

0.33.1 (2019-11-11)
-------------------
* resolved merge conflict
* Contributors: David Jablonski

0.33.0 (2019-10-10)
-------------------

0.32.2 (2019-09-09)
-------------------

0.32.1 (2019-08-08)
-------------------

0.32.0 (2019-07-06)
-------------------

0.31.0 (2019-06-07)
-------------------

0.30.0 (2019-05-20)
-------------------

0.29.2 (2019-03-06)
-------------------

0.29.1 (2019-03-03)
-------------------
* All: catkin lint files
* Contributors: Pierre Kancir

0.29.0 (2019-02-02)
-------------------
* Merge branch 'master' into param-timeout
* Contributors: Vladimir Ermakov

0.28.0 (2019-01-03)
-------------------

0.27.0 (2018-11-12)
-------------------

0.26.3 (2018-08-21)
-------------------

0.26.2 (2018-08-08)
-------------------

0.26.1 (2018-07-19)
-------------------

0.26.0 (2018-06-06)
-------------------

0.25.1 (2018-05-14)
-------------------

0.25.0 (2018-05-11)
-------------------

0.24.0 (2018-04-05)
-------------------

0.23.3 (2018-03-09)
-------------------

0.23.2 (2018-03-07)
-------------------

0.23.1 (2018-02-27)
-------------------

0.23.0 (2018-02-03)
-------------------

0.22.0 (2017-12-11)
-------------------

0.21.5 (2017-11-16)
-------------------

0.21.4 (2017-11-01)
-------------------

0.21.3 (2017-10-28)
-------------------

0.21.2 (2017-09-25)
-------------------

0.21.1 (2017-09-22)
-------------------

0.21.0 (2017-09-14)
-------------------

0.20.1 (2017-08-28)
-------------------

0.20.0 (2017-08-23)
-------------------

0.19.0 (2017-05-05)
-------------------
* cmake: remove Eigen warning
* Contributors: Vladimir Ermakov

0.18.7 (2017-02-24)
-------------------

0.18.6 (2017-02-07)
-------------------

0.18.5 (2016-12-12)
-------------------

0.18.4 (2016-11-11)
-------------------
* Update README for all packages
* Contributors: Vladimir Ermakov

0.18.3 (2016-07-07)
-------------------

0.18.2 (2016-06-30)
-------------------

0.18.1 (2016-06-24)
-------------------

0.18.0 (2016-06-23)
-------------------
* Test_mavros : fix compilation on gcc6.1
* Contributors: khancyr

0.17.3 (2016-05-20)
-------------------
* test `#546 <https://github.com/mavlink/mavros/issues/546>`_: Added check of control_toolbox version (1.14.0)
  In Kinetic control_toolbox changed API of Pid::initPid().
* Contributors: Vladimir Ermakov

0.17.2 (2016-04-29)
-------------------

0.17.1 (2016-03-28)
-------------------

0.17.0 (2016-02-09)
-------------------
* rebased with master
* Contributors: francois

0.16.6 (2016-02-04)
-------------------

0.16.5 (2016-01-11)
-------------------

0.16.4 (2015-12-14)
-------------------
* updated local position subscription topic
* Contributors: Andreas Antener

0.16.3 (2015-11-19)
-------------------

0.16.2 (2015-11-17)
-------------------

0.16.1 (2015-11-13)
-------------------

0.16.0 (2015-11-09)
-------------------

0.15.0 (2015-09-17)
-------------------
* test: update readme
* test: add required plugins
* test: new test for local_position + SSP (`#387 <https://github.com/mavlink/mavros/issues/387>`_) + URDF
* test: add schematic plane urdf
* Contributors: Vladimir Ermakov

0.14.2 (2015-08-20)
-------------------
* test: fix depend on angles, fix catkin lint warnings
* Contributors: Vladimir Ermakov

0.14.1 (2015-08-19)
-------------------

0.14.0 (2015-08-17)
-------------------
* test fix `#368 <https://github.com/mavlink/mavros/issues/368>`_: use mavros.setpoint module in demo
* test: `#368 <https://github.com/mavlink/mavros/issues/368>`_: initial import of setpoint_demo.py
* test: Fix library name.
* test_mavros: pid_controller: declare PID variables as local
* test_mavros: move headers to include/test_mavros and setup for install
* test_mavros: removed pid_controller as lib; instantiate object so to use on offboard test
* test_mavros: CMakeLists: small ident correction
* test_mavros: pid_controller: include <array> so to make Travis happy
* test_mavros: added PID controller utility for velocity control on tests
* test_mavros: changed test_type to test_setup; namespace also
* Contributors: TSC21, Vladimir Ermakov

0.13.1 (2015-08-05)
-------------------
* test: add link to APM sitl video
* test_mavros: put acceleration note out of title
* Minor titles correction
* test_mavros: update README.md with tutorial to use PX4 ROS SITL
* Contributors: TSC21, Vladimir Ermakov

0.13.0 (2015-08-01)
-------------------
* Update iris_empty_world_offboard_ctl.launch
* test: fix prerelease building
* test: move launch
* sitl_tests: turn pos_setpoint code more elegant
* sitl_tests: minor code tweak; use angles.h package
* sitl_tests: offboard_control: included array lib; init threshold in constructor
* sitl_tests: added normal distribution position error threshold generator
* sitl_tests: add eigen dependency to CMakeLists and package.xml
* sitl_tests: "eigenize" offboard_control code; generalize offb control launch file
* sitl_tests: added px4 and rotors_simulator packages to package.xml dependencies
* sitl_tests: define `sitl_tests` group; change `tgt_component` to 1
* sitl_tests: offboard_mode: minor code refining
* sitl_tests: code cleaning
* sitl_tests: uncrustify code
* sitl_tests: offboard_control: velocity: added eight and ellipse-shaped paths
* sitl_tests: offboard_control: velocity: added circle-shaped path
* sitl_tests: added offboard velocity control - square shaped path for now
* sitl_tests: offboard_control: added ellipse-shaped path
* sitl_tests: offboard_control: added circle-shaped path
* sitl_tests: generalize offboard posctl so it can handle vel/accel control; added support to "eight" sphaped path
* sitl_tests: added base node
* sitl_test: added integrated launch file for OFFB POSCTL square shape
* sitl_tests: turn sitl_test_node as generic node to both APM and PX4
* sitl_tests: test structure definition; first working test routine
* test: import launch for imu testing
* test: apm sitl and imu test reproduction steps
* test: Add test_marvros package stub
* Contributors: TSC21, Vladimir Ermakov, wangsen1312
