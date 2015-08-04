/**
 * Test libmavros frame conversion utilities
 */

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <mavros/mavros_uas.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using mavros::UAS;

static const double epsilon = 1e-9;
static const double epsilon_f = 1e-6;
// gMock has ability to define array matcher, but there problems with that.
// so trying good old for loop

#define ASSERT_QUATERNION(q1, q2, epsilon)	\
	ASSERT_NEAR(q1.w(), q2.w(), epsilon);	\
	ASSERT_NEAR(q1.x(), q2.x(), epsilon);	\
	ASSERT_NEAR(q1.y(), q2.y(), epsilon);	\
	ASSERT_NEAR(q1.z(), q2.z(), epsilon)

#define EXPECT_QUATERNION(q1, q2, epsilon)	\
	EXPECT_NEAR(q1.w(), q2.w(), epsilon);	\
	EXPECT_NEAR(q1.x(), q2.x(), epsilon);	\
	EXPECT_NEAR(q1.y(), q2.y(), epsilon);	\
	EXPECT_NEAR(q1.z(), q2.z(), epsilon)

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

	EXPECT_QUATERNION(expected, out, epsilon);
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
	auto eigen_neg_q = UAS::quaternion_from_rpy(-1.0, -2.0, -3.0);
	tf2::Quaternion bt_q; bt_q.setRPY(1.0, 2.0, 3.0);
	tf2::Quaternion bt_neg_q; bt_neg_q.setRPY(-1.0, -2.0, -3.0);

	EXPECT_QUATERNION(bt_q, eigen_q, epsilon);
	EXPECT_QUATERNION(bt_neg_q, eigen_neg_q, epsilon);
}

TEST(UAS, quaternion_from_rpy__paranoic_check)
{
	auto q1 = UAS::quaternion_from_rpy(1.0, 2.0, 3.0);
	auto q2 = UAS::quaternion_from_rpy(Eigen::Vector3d(1.0, 2.0, 3.0));

	EXPECT_QUATERNION(q1, q2, epsilon);
}

TEST(UAS, quaternion_to_rpy__123)
{
	auto q = UAS::quaternion_from_rpy(1.0, 2.0, 3.0);
	auto rpy = UAS::quaternion_to_rpy(q);

	EXPECT_NEAR(1.0, rpy.x(), epsilon);
	EXPECT_NEAR(2.0, rpy.y(), epsilon);
	EXPECT_NEAR(3.0, rpy.z(), epsilon);
}

// test fails now.
TEST(UAS, quaternion_to_rpy__123_negative)
{
	auto q = UAS::quaternion_from_rpy(-1.0, -2.0, -3.0);
	auto rpy = UAS::quaternion_to_rpy(q);

	EXPECT_NEAR(-1.0, rpy.x(), epsilon);
	EXPECT_NEAR(-2.0, rpy.y(), epsilon);
	EXPECT_NEAR(-3.0, rpy.z(), epsilon);
}

TEST(UAS, quaternion_to_rpy__check_compatibility)
{
	auto eigen_q = UAS::quaternion_from_rpy(1.0, 2.0, 3.0);
	auto eigen_neg_q = UAS::quaternion_from_rpy(-1.0, -2.0, -3.0);
	tf2::Quaternion bt_q; bt_q.setRPY(1.0, 2.0, 3.0);
	tf2::Quaternion bt_neg_q; bt_neg_q.setRPY(-1.0, -2.0, -3.0);

	// ensure that quaternions are equal
	ASSERT_QUATERNION(bt_q, eigen_q, epsilon);
	ASSERT_QUATERNION(bt_neg_q, eigen_neg_q, epsilon);

	tf2::Matrix3x3 bt_m(bt_q);
	double bt_roll, bt_pitch, bt_yaw;
	bt_m.getRPY(bt_roll, bt_pitch, bt_yaw);
	auto eigen_rpy = UAS::quaternion_to_rpy(eigen_q);

	// fails
	EXPECT_NEAR(bt_roll, eigen_rpy.x(), epsilon);
	EXPECT_NEAR(bt_pitch, eigen_rpy.y(), epsilon);
	EXPECT_NEAR(bt_yaw, eigen_rpy.z(), epsilon);

	tf2::Matrix3x3 bt_neg_m(bt_neg_q);
	double bt_neg_roll, bt_neg_pitch, bt_neg_yaw;
	bt_neg_m.getRPY(bt_neg_roll, bt_neg_pitch, bt_neg_yaw);
	auto eigen_neg_rpy = UAS::quaternion_to_rpy(eigen_neg_q);

	// ok
	EXPECT_NEAR(bt_neg_roll, eigen_neg_rpy.x(), epsilon);
	EXPECT_NEAR(bt_neg_pitch, eigen_neg_rpy.y(), epsilon);
	EXPECT_NEAR(bt_neg_yaw, eigen_neg_rpy.z(), epsilon);

	// fails!
	//EXPECT_NEAR(-1.0, bt_neg_roll, epsilon);
	//EXPECT_NEAR(-2.0, bt_neg_pitch, epsilon);
	//EXPECT_NEAR(-3.0, bt_neg_yaw, epsilon);
}

TEST(UAS, getYaw__123)
{
	auto q = UAS::quaternion_from_rpy(1.0, 2.0, 3.0);

	EXPECT_NEAR(3.0, UAS::getYaw(q), epsilon);
}

#if 0
TEST(UAS, quaternion_from_rpy__getYaw__issue_358)
{
	// test for issue #358
	auto q1 = UAS::quaternion_from_rpy(0.0, 0.0, M_PI);
	auto q2 = Eigen::Quaterniond(-1.0, 0.0, 0.0, 0.0);
	auto q1_ned = UAS::transform_frame_enu_ned(q1);
	auto yaw = UAS::getYaw(q1);
	auto yaw_ned = UAS::getYaw(q1_ned);
	auto rpy = UAS::quaternion_to_rpy(q1);
	auto rpy_ned = UAS::quaternion_to_rpy(q1_ned);

	std::cout << "Q1: (" << q1.w() << " " << q1.x() << " " << q1.y() << " " << q1.z() << ")\n";
	std::cout << "Q2: (" << q2.w() << " " << q2.x() << " " << q2.y() << " " << q2.z() << ")\n";
	std::cout << "Q1 NED: (" << q1_ned.w() << " " << q1_ned.x() << " " << q1_ned.y() << " " << q1_ned.z() << ")\n";
	std::cout << "YAW: " << yaw << " NED: " << yaw_ned << "\n";
	std::cout << "RPY: " << rpy << "\n";
	std::cout << "RPY NED:" << rpy_ned << "\n";

	//EXPECT_QUATERNION(q1, q2, epsilon);

	EXPECT_NEAR(-M_PI, yaw, epsilon);
	EXPECT_NEAR(M_PI, yaw_ned, epsilon);
}
#endif

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
