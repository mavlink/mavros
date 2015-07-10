/**
 * Test libmavros frame conversion utilities
 */

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <mavros/mavros_uas.h>

using mavros::UAS;

static const double epsilon = 1e-6;

/* -*- test general transform function -*- */

TEST(UAS, transform_frame__vector3d_123)
{
	Eigen::Vector3d input(1, 2, 3);
	Eigen::Vector3d expected(1, -2, -3);

	auto out = UAS::transform_frame(input);

	EXPECT_NEAR(expected.x(), out.x(), epsilon);
	EXPECT_NEAR(expected.y(), out.y(), epsilon);
	EXPECT_NEAR(expected.z(), out.z(), epsilon);
}

TEST(UAS, transform_frame__quaterniond_123)
{
	auto input = UAS::quaternion_from_rpy(1.0, 2.0, 3.0);
	auto expected = UAS::quaternion_from_rpy(1.0, -2.0, -3.0);

	auto out = UAS::transform_frame(input);

	EXPECT_NEAR(expected.w(), out.w(), epsilon);
	EXPECT_NEAR(expected.x(), out.x(), epsilon);
	EXPECT_NEAR(expected.y(), out.y(), epsilon);
	EXPECT_NEAR(expected.z(), out.z(), epsilon);
}

TEST(UAS, transform_frame__covariance3x3)
{
}

TEST(UAS,  transform_frame__covariance6x6)
{
	UAS::Covariance6x6 test = {
		176.75,    1E-6,      1E-6,  1E-6,   4.14,  1E-6,
		1E-6,    1748.8,      1E-6,   2.1,   1E-6,  1E-6,
		1E-6,      1E-6,  11447.14,  1E-6,   4.17,  1E-6,
		1E-6,       2.1,      1E-6, 10001,   1E-6,  1E-6,
		4.14,      1E-6,      4.17,  1E-6,  14111,  1E-6,
		1E-6,      1E-6,      1E-6,  1E-6,   1E-6,   101
	};

	UAS::Covariance6x6 exp_out = { // XXX Still have to confirm this output
		176.75,    -1E-6,      -1E-6,  -1E-6,   -4.14,  -1E-6,
		-1E-6,    1748.8,      -1E-6,   -2.1,   -1E-6,   1E-6,
		-1E-6,     -1E-6,   11447.14,   1E-6,    4.17,   1E-6,
		-1E-6,      -2.1,       1E-6,  10001,    1E-6,   1E-6,
		-4.14,      1E-6,       4.17,   1E-6,   14111,   1E-6,
		-1E-6,      1E-6,       1E-6,   1E-6,    1E-6,    101
	};

	auto input = test;
	auto expected_out = exp_out;

	auto out = UAS::transform_frame(input);

	//EXPECT_NEAR(expected_out, out, epsilon);
	EXPECT_EQ(expected_out, out);
}

TEST(UAS, quaternion_from_rpy__check_compatibility)
{
	auto eigen_q = UAS::quaternion_from_rpy(1.0, 2.0, 3.0);
	auto bt_q = tf::createQuaternionFromRPY(1.0, 2.0, 3.0);

	EXPECT_NEAR(bt_q.w(), eigen_q.w(), epsilon);
	EXPECT_NEAR(bt_q.x(), eigen_q.x(), epsilon);
	EXPECT_NEAR(bt_q.y(), eigen_q.y(), epsilon);
	EXPECT_NEAR(bt_q.z(), eigen_q.z(), epsilon);
}

TEST(UAS, quaternion_from_rpy__paranoic_check)
{
	auto q1 = UAS::quaternion_from_rpy(1.0, 2.0, 3.0);
	auto q2 = UAS::quaternion_from_rpy(Eigen::Vector3d(1.0, 2.0, 3.0));

	EXPECT_NEAR(q1.w(), q2.w(), epsilon);
	EXPECT_NEAR(q1.x(), q2.x(), epsilon);
	EXPECT_NEAR(q1.y(), q2.y(), epsilon);
	EXPECT_NEAR(q1.z(), q2.z(), epsilon);
}

TEST(UAS, quaternion_to_rpy__123)
{
	auto q = UAS::quaternion_from_rpy(1.0, 2.0, 3.0);
	auto rpy = UAS::quaternion_to_rpy(q);

	EXPECT_NEAR(1.0, rpy.x(), epsilon);
	EXPECT_NEAR(2.0, rpy.y(), epsilon);
	EXPECT_NEAR(3.0, rpy.z(), epsilon);
}

/* -*- test covariance transform -*- */


int main(int argc, char **argv)
{
	//ros::init(argc, argv, "mavros_test", ros::init_options::AnonymousName);
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
