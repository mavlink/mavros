/**
 * Test libmavros frame conversion utilities
 */

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <mavros/mavros_uas.h>

#include <tf2/LinearMath/Quaternion.h>

using mavros::UAS;

static const double epsilon = 1e-9;
static const double epsilon_f = 1e-6;
// gMock has ability to define array matcher, but there problems with that.
// so trying good old for loop

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
	UAS::Covariance3d input = {{
		1.0, 2.0, 3.0,
		4.0, 5.0, 6.0,
		7.0, 8.0, 9.0
	}};

	/* Calculated as:
	 *         | 1  0  0 |
	 * input * | 0 -1  0 |
	 *         | 0  0 -1 |
	 */
	UAS::Covariance3d expected = {{
		1.0, -2.0, -3.0,
		4.0, -5.0, -6.0,
		7.0, -8.0, -9.0
	}};

	auto out = UAS::transform_frame(input);

	for (size_t idx = 0; idx < expected.size(); idx++) {
		SCOPED_TRACE(idx);
		EXPECT_NEAR(expected[idx], out[idx], epsilon);
	}
}

#if 0
// not implemented
TEST(UAS,  transform_frame__covariance6x6)
{
	UAS::Covariance6d input = {{
		 1.0,  2.0,  3.0,  4.0,  5.0,  6.0,
		 7.0,  8.0,  9.0, 10.0, 11.0, 12.0,
		13.0, 14.0, 15.0, 16.0, 17.0, 18.0,
		19.0, 20.0, 21.0, 22.0, 23.0, 24.0,
		25.0, 26.0, 27.0, 28.0, 29.0, 30.0,
		31.0, 32.0, 33.0, 34.0, 35.0, 36.0
	}};

	UAS::Covariance6d expected = {{
		 1.0,  -2.0,  -3.0,  4.0,  -5.0,  -6.0,
		 7.0,  -8.0,  -9.0, 10.0, -11.0, -12.0,
		13.0, -14.0, -15.0, 16.0, -17.0, -18.0,
		19.0, -20.0, -21.0, 22.0, -23.0, -24.0,
		25.0, -26.0, -27.0, 28.0, -29.0, -30.0,
		31.0, -32.0, -33.0, 34.0, -35.0, -36.0
	}};

	auto out = UAS::transform_frame(input);

	for (size_t idx = 0; idx < expected.size(); idx++) {
		SCOPED_TRACE(idx);
		EXPECT_NEAR(expected[idx], out[idx], epsilon);
	}
}
#endif

/* -*- quaternion_from_rpy / getYaw -*- */

TEST(UAS, quaternion_from_rpy__check_compatibility)
{
	auto eigen_q = UAS::quaternion_from_rpy(1.0, 2.0, 3.0);
	//auto bt_q = tf::createQuaternionFromRPY(1.0, 2.0, 3.0);	// TF1
	tf2::Quaternion bt_q; bt_q.setRPY(1.0, 2.0, 3.0);		// TF2

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

TEST(UAS, getYaw__123)
{
	auto q = UAS::quaternion_from_rpy(1.0, 2.0, 3.0);

	EXPECT_NEAR(3.0, UAS::getYaw(q), epsilon);
}

/* -*- mavlink util -*- */

TEST(UAS, quaternion_to_mavlink__123)
{
	auto eigen_q = UAS::quaternion_from_rpy(1.0, 2.0, 3.0);
	float mavlink_q[4];

	UAS::quaternion_to_mavlink(eigen_q, mavlink_q);

	EXPECT_NEAR(mavlink_q[0], eigen_q.w(), epsilon_f);
	EXPECT_NEAR(mavlink_q[1], eigen_q.x(), epsilon_f);
	EXPECT_NEAR(mavlink_q[2], eigen_q.y(), epsilon_f);
	EXPECT_NEAR(mavlink_q[3], eigen_q.z(), epsilon_f);
}


int main(int argc, char **argv)
{
	//ros::init(argc, argv, "mavros_test", ros::init_options::AnonymousName);
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
