^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_mavros
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
