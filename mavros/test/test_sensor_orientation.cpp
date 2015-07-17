/**
 * Test libmavros sensor orientation utilities
 *
 * We look up for some rotations, not fill list.
 */

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <mavros/mavros_uas.h>

using mavros::UAS;

static const double epsilon = 1e-9;

#define EXPECT_QUATERNION(exp, res) \
	EXPECT_NEAR(exp.w(), res.w(), epsilon); \
	EXPECT_NEAR(exp.x(), res.x(), epsilon); \
	EXPECT_NEAR(exp.y(), res.y(), epsilon); \
	EXPECT_NEAR(exp.z(), res.z(), epsilon)

TEST(UAS, sensor_orientation_matching__none)
{
	auto expected = UAS::quaternion_from_rpy(0.0, 0.0, 0.0);
	auto out = UAS::sensor_orientation_matching(MAV_SENSOR_ROTATION_NONE);

	EXPECT_QUATERNION(expected, out);
}

TEST(UAS, sensor_orientation_matching__roll_180)
{
	auto expected = UAS::quaternion_from_rpy(M_PI, 0.0, 0.0);
	auto out = UAS::sensor_orientation_matching(MAV_SENSOR_ROTATION_ROLL_180);

	EXPECT_QUATERNION(expected, out);
}

TEST(UAS, sensor_orientation_matching__roll_180_yaw_90)
{
	auto expected = UAS::quaternion_from_rpy(M_PI, 0.0, M_PI/2);
	auto out = UAS::sensor_orientation_matching(MAV_SENSOR_ROTATION_ROLL_180_YAW_90);

	EXPECT_QUATERNION(expected, out);
}


int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
