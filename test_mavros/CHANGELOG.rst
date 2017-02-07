^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_mavros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.17.5 (2017-02-07)
-------------------

0.17.4 (2016-06-23)
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
