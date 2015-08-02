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

TEST(UAS, str_sensor_orientation__none)
{
	EXPECT_EQ("NONE", UAS::str_sensor_orientation(MAV_SENSOR_ROTATION_NONE));
}

TEST(UAS, str_sensor_orientation__roll_180)
{
	EXPECT_EQ("ROLL_180", UAS::str_sensor_orientation(MAV_SENSOR_ROTATION_ROLL_180));
}

TEST(UAS, str_sensor_orientation__roll_180_yaw_90)
{
	EXPECT_EQ("ROLL_180_YAW_90", UAS::str_sensor_orientation(MAV_SENSOR_ROTATION_ROLL_180_YAW_90));
}

TEST(UAS, orientation_from_str__none)
{
	EXPECT_EQ(MAV_SENSOR_ROTATION_NONE, UAS::orientation_from_str("NONE"));
}

TEST(UAS, orientation_from_str__unknown)
{
	EXPECT_LT(UAS::orientation_from_str("completely wrong identificator"), 0);
}

TEST(UAS, orientation_from_str__number)
{
	EXPECT_EQ(MAV_SENSOR_ROTATION_ROLL_270, UAS::orientation_from_str("20"));
}

TEST(UAS, orientation_from_str__wrong_number)
{
	// 123 >> 38 (max)
	EXPECT_LT(UAS::orientation_from_str("123"), 0);
}

TEST(UAS, orientation_from_str__roll_180)
{
	EXPECT_EQ(MAV_SENSOR_ROTATION_ROLL_180, UAS::orientation_from_str("ROLL_180"));
}

TEST(UAS, orientation_from_str__roll_180_yaw_90)
{
	EXPECT_EQ(MAV_SENSOR_ROTATION_ROLL_180_YAW_90, UAS::orientation_from_str("ROLL_180_YAW_90"));
}

TEST(UAS, orientation_from_str__last_element_rpy_315)
{
	EXPECT_EQ(MAV_SENSOR_ROTATION_ROLL_315_PITCH_315_YAW_315, UAS::orientation_from_str("ROLL_315_PITCH_315_YAW_315"));
}

int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
