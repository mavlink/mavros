/**
 * Test libmavros frame conversion utilities
 */

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <mavros/mavros_uas.h>

using mavros::UAS;


/* -*- test general Vector3 transform function -*- */

TEST(VECTOR, transform_frame_xyz_111)
{
	tf::Vector3 input(1, 1, 1);
	tf::Vector3 expected_out(1, -1, -1);

	auto out = UAS::transform_frame_xyz(input.x(), input.y(), input.z());
	EXPECT_EQ(expected_out, out);
}

TEST(VECTOR, transform_frame_xyz_pi00)
{
	tf::Vector3 input(M_PI, 0, 0);
	tf::Vector3 expected_out(M_PI, 0, 0);

	auto out = UAS::transform_frame_xyz(input.x(), input.y(), input.z());
	EXPECT_EQ(expected_out, out);
}

TEST(VECTOR, transform_frame_xyz_0pi0)
{
	tf::Vector3 input(0, M_PI, 0);
	tf::Vector3 expected_out(0, -M_PI, 0);

	auto out = UAS::transform_frame_xyz(input.x(), input.y(), input.z());
	EXPECT_EQ(expected_out, out);
}

TEST(VECTOR, transform_frame_xyz_00pi)
{
	tf::Vector3 input(0, 0, M_PI);
	tf::Vector3 expected_out(0, 0, -M_PI);

	auto out = UAS::transform_frame_xyz(input.x(), input.y(), input.z());
	EXPECT_EQ(expected_out, out);
}


/* -*- test attitude RPY transform -*- */

TEST(VECTOR, transform_frame_attitude_rpy_111)
{
	tf::Vector3 input(1, 1, 1);
	tf::Vector3 expected_out(0, -1, -1);

	auto out = UAS::transform_frame_attitude_rpy(input.x(), input.y(), input.z());
	EXPECT_EQ(expected_out, out);
}

TEST(VECTOR, transform_frame_attitude_rpy_pi00)
{
	tf::Vector3 input(M_PI, 0, 0);
	tf::Vector3 expected_out(M_PI, 0, 0);

	auto out = UAS::transform_frame_attitude_rpy(input.x(), input.y(), input.z());
	EXPECT_EQ(expected_out, out);
}

TEST(VECTOR, transform_frame_attitude_rpy_0pi0)
{
	tf::Vector3 input(0, M_PI, 0);
	tf::Vector3 expected_out(0, M_PI, 0);

	auto out = UAS::transform_frame_attitude_rpy(input.x(), input.y(), input.z());
	EXPECT_EQ(expected_out, out);
}

TEST(VECTOR, transform_frame_attitude_rpy_00pi)
{
	tf::Vector3 input(0, 0, M_PI);
	tf::Vector3 expected_out(0, 0, M_PI);

	auto out = UAS::transform_frame_attitude_rpy(input.x(), input.y(), input.z());
	EXPECT_EQ(expected_out, out);
}




int main(int argc, char **argv)
{
	//ros::init(argc, argv, "mavros_test", ros::init_options::AnonymousName);
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
