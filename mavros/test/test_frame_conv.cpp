/**
 * Test libmavros frame conversion utilities
 */

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <mavros/mavros_uas.h>

using mavros::UAS;


static void log_vectors(tf::Vector3 &in, tf::Vector3 &out, tf::Vector3 &expected)
{
	ROS_INFO("In (x y z): %f %f %f", in.x(), in.y(), in.z());
	ROS_INFO("Expected:   %f %f %f", expected.x(), expected.y(), expected.z());
	ROS_INFO("Out:        %f %f %f", out.x(), out.y(), out.z());
}

static void log_quaternion(tf::Quaternion &in, tf::Quaternion &out, tf::Quaternion &expected)
{
	ROS_INFO("In (x y z w): %f %f %f %f", in.x(), in.y(), in.z(), in.w());
	ROS_INFO("Expected:     %f %f %f %f", expected.x(), expected.y(), expected.z(), expected.w());
	ROS_INFO("Out:          %f %f %f %f", out.x(), out.y(), out.z(), out.w());
}

/* -*- test general Vector3 transform function -*- */

TEST(VECTOR, transform_frame_xyz_100)
{
	tf::Vector3 input(1, 0, 0);
	tf::Vector3 expected_out(1, 0, 0);

	auto out = UAS::transform_frame_xyz(input.x(), input.y(), input.z());

	log_vectors(input, out, expected_out);
	EXPECT_EQ(expected_out, out);
}

TEST(VECTOR, transform_frame_xyz_110)
{
	tf::Vector3 input(1, 1, 0);
	tf::Vector3 expected_out(1, -1, 0);

	auto out = UAS::transform_frame_xyz(input.x(), input.y(), input.z());

	log_vectors(input, out, expected_out);
	EXPECT_EQ(expected_out, out);
}

TEST(VECTOR, transform_frame_xyz_111)
{
	tf::Vector3 input(1, 1, 1);
	tf::Vector3 expected_out(1, -1, -1);

	auto out = UAS::transform_frame_xyz(input.x(), input.y(), input.z());

	log_vectors(input, out, expected_out);
	EXPECT_EQ(expected_out, out);
}

/* -*- test attitude RPY transform -*- */

TEST(VECTOR, transform_frame_attitude_rpy_pi00)
{
	tf::Vector3 input(M_PI, 0, 0);
	tf::Vector3 expected_out(2 * M_PI, 0, 0);

	auto out = UAS::transform_frame_attitude_rpy(input.x(), input.y(), input.z());

	log_vectors(input, out, expected_out);
	EXPECT_EQ(expected_out, out);
}

TEST(VECTOR, transform_frame_attitude_rpy_0pi0)
{
	tf::Vector3 input(0, M_PI, 0);
	tf::Vector3 expected_out(M_PI, M_PI, 0);

	auto out = UAS::transform_frame_attitude_rpy(input.x(), input.y(), input.z());

	log_vectors(input, out, expected_out);
	EXPECT_EQ(expected_out, out);
}

TEST(VECTOR, transform_frame_attitude_rpy_00pi)
{
	tf::Vector3 input(0, 0, M_PI);
	tf::Vector3 expected_out(M_PI, 0, M_PI);

	auto out = UAS::transform_frame_attitude_rpy(input.x(), input.y(), input.z());

	log_vectors(input, out, expected_out);
	EXPECT_EQ(expected_out, out);
}

/* -*- test attitude quaternion transform -*- */

TEST(QUATERNION,  transform_frame_attitude_q_pi00)
{
	auto input = tf::createQuaternionFromRPY(M_PI, 0, 0);
	auto expected_out = tf::createQuaternionFromRPY(2 * M_PI, 0, 0);

	auto out = UAS::transform_frame_attitude_q(input);

	log_quaternion(input, out, expected_out);
	EXPECT_EQ(expected_out, out);
}

TEST(QUATERNION,  transform_frame_attitude_q_0pi0)
{
	auto input = tf::createQuaternionFromRPY(0, M_PI, 0);
	auto expected_out = tf::createQuaternionFromRPY(M_PI, M_PI, 0);

	auto out = UAS::transform_frame_attitude_q(input);

	log_quaternion(input, out, expected_out);
	EXPECT_EQ(expected_out, out);
}

TEST(QUATERNION,  transform_frame_attitude_q_00pi)
{
	auto input = tf::createQuaternionFromRPY(0, 0, M_PI);
	auto expected_out = tf::createQuaternionFromRPY(M_PI, 0, M_PI);

	auto out = UAS::transform_frame_attitude_q(input);

	log_quaternion(input, out, expected_out);
	EXPECT_EQ(expected_out, out);
}


int main(int argc, char **argv)
{
	//ros::init(argc, argv, "mavros_test", ros::init_options::AnonymousName);
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
