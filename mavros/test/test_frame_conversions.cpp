/**
 * Test libmavros frame conversion utilities
 */

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <mavros/frame_tf.h>

using namespace mavros;

static const double epsilon = 1e-9;
static const double epsilon_f = 1e-6;
// gMock has ability to define array matcher, but there problems with that.
// so trying good old for loop

#define EXPECT_QUATERNION(q1, q2, epsilon)	\
	EXPECT_NEAR(q1.w(), q2.w(), epsilon);	\
	EXPECT_NEAR(q1.x(), q2.x(), epsilon);	\
	EXPECT_NEAR(q1.y(), q2.y(), epsilon);	\
	EXPECT_NEAR(q1.z(), q2.z(), epsilon)

/* -*- test general transform function -*- */

TEST(FRAME_TF, transform_static_frame__aircraft_to_baselink_123)
{
	Eigen::Vector3d input(1, 2, 3);
	Eigen::Vector3d expected(1, -2, -3);

	auto out = ftf::detail::transform_static_frame(input, ftf::StaticTF::AIRCRAFT_TO_BASELINK);

	EXPECT_NEAR(expected.x(), out.x(), epsilon);
	EXPECT_NEAR(expected.y(), out.y(), epsilon);
	EXPECT_NEAR(expected.z(), out.z(), epsilon);
}

TEST(FRAME_TF, transform_static_frame__baselink_to_aircraft_123)
{
	Eigen::Vector3d input(1, 2, 3);
	Eigen::Vector3d expected(1, -2, -3);

	auto out = ftf::detail::transform_static_frame(input, ftf::StaticTF::BASELINK_TO_AIRCRAFT);

	EXPECT_NEAR(expected.x(), out.x(), epsilon);
	EXPECT_NEAR(expected.y(), out.y(), epsilon);
	EXPECT_NEAR(expected.z(), out.z(), epsilon);
}

TEST(FRAME_TF, transform_static_frame__enu_to_ned_123)
{
	Eigen::Vector3d input(1, 2, 3);
	Eigen::Vector3d expected(2, 1, -3);

	auto out = ftf::detail::transform_static_frame(input, ftf::StaticTF::ENU_TO_NED);

	EXPECT_NEAR(expected.x(), out.x(), epsilon);
	EXPECT_NEAR(expected.y(), out.y(), epsilon);
	EXPECT_NEAR(expected.z(), out.z(), epsilon);
}

TEST(FRAME_TF, transform_static_frame__ned_to_enu_123)
{
	Eigen::Vector3d input(1, 2, 3);
	Eigen::Vector3d expected(2, 1, -3);

	auto out = ftf::detail::transform_static_frame(input, ftf::StaticTF::NED_TO_ENU);

	EXPECT_NEAR(expected.x(), out.x(), epsilon);
	EXPECT_NEAR(expected.y(), out.y(), epsilon);
	EXPECT_NEAR(expected.z(), out.z(), epsilon);
}

TEST(FRAME_TF, transform_static_frame__ecef_to_enu_123_00)
{
	Eigen::Vector3d input(1, 2, 3);
	Eigen::Vector3d map_origin(0, 0, 0);
	Eigen::Vector3d expected(2, 3, 1);

	auto out = ftf::detail::transform_static_frame(input, map_origin, ftf::StaticTF::ECEF_TO_ENU);

	EXPECT_NEAR(expected.x(), out.x(), epsilon);
	EXPECT_NEAR(expected.y(), out.y(), epsilon);
	EXPECT_NEAR(expected.z(), out.z(), epsilon);
}

TEST(FRAME_TF, transform_static_frame__enu_to_ecef_123_00)
{
	Eigen::Vector3d input(1, 2, 3);
	Eigen::Vector3d map_origin(0, 0, 0);
	Eigen::Vector3d expected(3, 1, 2);

	auto out = ftf::detail::transform_static_frame(input, map_origin, ftf::StaticTF::ENU_TO_ECEF);

	EXPECT_NEAR(expected.x(), out.x(), epsilon);
	EXPECT_NEAR(expected.y(), out.y(), epsilon);
	EXPECT_NEAR(expected.z(), out.z(), epsilon);
}

TEST(FRAME_TF, transform_static_frame__ecef_to_enu_123_4030)
{
	Eigen::Vector3d input(1, 2, 3);
	Eigen::Vector3d map_origin(40, 30, 0);
	Eigen::Vector3d expected(1.23205080756887, 1.09867532044397, 3.35782122034753);

	auto out = ftf::detail::transform_static_frame(input, map_origin, ftf::StaticTF::ECEF_TO_ENU);

	EXPECT_NEAR(expected.x(), out.x(), epsilon);
	EXPECT_NEAR(expected.y(), out.y(), epsilon);
	EXPECT_NEAR(expected.z(), out.z(), epsilon);
}

TEST(FRAME_TF, transform_static_frame__enu_to_ecef_123_4030)
{
	Eigen::Vector3d input(1, 2, 3);
	Eigen::Vector3d map_origin(40, 30, 0);
	Eigen::Vector3d expected(0.3769010460539777, 1.37230445877637, 3.46045171529757);

	auto out = ftf::detail::transform_static_frame(input, map_origin, ftf::StaticTF::ENU_TO_ECEF);

	EXPECT_NEAR(expected.x(), out.x(), epsilon);
	EXPECT_NEAR(expected.y(), out.y(), epsilon);
	EXPECT_NEAR(expected.z(), out.z(), epsilon);
}

TEST(FRAME_TF, quaternion_transforms__ned_to_ned_123)
{
	auto input_aircraft_ned_orient = ftf::quaternion_from_rpy(1.0, 2.0, 3.0);
	auto aircraft_enu_orient = ftf::transform_orientation_ned_enu(input_aircraft_ned_orient);
	auto baselink_enu_orient = ftf::transform_orientation_aircraft_baselink(aircraft_enu_orient);
	aircraft_enu_orient = ftf::transform_orientation_baselink_aircraft(baselink_enu_orient);
	auto output_aircraft_ned = ftf::transform_orientation_enu_ned(aircraft_enu_orient);

	EXPECT_QUATERNION(input_aircraft_ned_orient, output_aircraft_ned, epsilon);
}

#if 0
// not implemented
TEST(FRAME_TF, transform_static_frame__quaterniond_123)
{
	auto input = ftf::quaternion_from_rpy(1.0, 2.0, 3.0);
	auto expected = ftf::quaternion_from_rpy(1.0, -2.0, -3.0);

	ftf::TRANSFORM_TYPE enu_sensor = ftf::PLATFORM_TO_ENU;
	auto out = ftf::transform_frame(input,enu_sensor);

	EXPECT_QUATERNION(expected, out, epsilon);
}

TEST(FRAME_TF, transform_frame__covariance3x3)
{
	ftf::Covariance3d input = {{
		1.0, 2.0, 3.0,
		4.0, 5.0, 6.0,
		7.0, 8.0, 9.0
	}};

	/* Calculated as:
	 *         | 1  0  0 |
	 * input * | 0 -1  0 |
	 *         | 0  0 -1 |
	 */
	ftf::Covariance3d expected = {{
		1.0, -2.0, -3.0,
		4.0, -5.0, -6.0,
		7.0, -8.0, -9.0
	}};

	auto out = ftf::transform_frame(input,enu_sensor);

	for (size_t idx = 0; idx < expected.size(); idx++) {
		SCOPED_TRACE(idx);
		EXPECT_NEAR(expected[idx], out[idx], epsilon);
	}
}
TEST(FRAME_TF,  transform_frame__covariance6x6)
{
	ftf::Covariance6d input = {{
		 1.0,  2.0,  3.0,  4.0,  5.0,  6.0,
		 7.0,  8.0,  9.0, 10.0, 11.0, 12.0,
		13.0, 14.0, 15.0, 16.0, 17.0, 18.0,
		19.0, 20.0, 21.0, 22.0, 23.0, 24.0,
		25.0, 26.0, 27.0, 28.0, 29.0, 30.0,
		31.0, 32.0, 33.0, 34.0, 35.0, 36.0
	}};

	ftf::Covariance6d expected = {{
		 1.0,  -2.0,  -3.0,  4.0,  -5.0,  -6.0,
		 7.0,  -8.0,  -9.0, 10.0, -11.0, -12.0,
		13.0, -14.0, -15.0, 16.0, -17.0, -18.0,
		19.0, -20.0, -21.0, 22.0, -23.0, -24.0,
		25.0, -26.0, -27.0, 28.0, -29.0, -30.0,
		31.0, -32.0, -33.0, 34.0, -35.0, -36.0
	}};

	auto out = ftf::transform_frame(input);

	for (size_t idx = 0; idx < expected.size(); idx++) {
		SCOPED_TRACE(idx);
		EXPECT_NEAR(expected[idx], out[idx], epsilon);
	}
}
#endif



int main(int argc, char **argv)
{
	//ros::init(argc, argv, "mavros_test", ros::init_options::AnonymousName);
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
