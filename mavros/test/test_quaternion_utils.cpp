/**
 * Test libmavros quaternion utilities
 */

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <mavros/mavros_uas.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using mavros::UAS;

static const double epsilon = 1e-9;
static const double epsilon_f = 1e-6;
static const double deg_to_rad = M_PI / 180.0;
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
	// this test only works on positive rpy: 0..pi
	auto q = UAS::quaternion_from_rpy(1.0, 2.0, 3.0);
	auto rpy = UAS::quaternion_to_rpy(q);

	EXPECT_NEAR(1.0, rpy.x(), epsilon);
	EXPECT_NEAR(2.0, rpy.y(), epsilon);
	EXPECT_NEAR(3.0, rpy.z(), epsilon);
}

TEST(UAS, quaternion_to_rpy__pm_pi)
{
	// this test try large count of different angles

	// in degrees
	const ssize_t min = -180;
	const ssize_t max = 180;
	const ssize_t step = 15;

	const auto test_orientation = UAS::quaternion_from_rpy(1.0, 2.0, 3.0);

	for (ssize_t roll = min; roll <= max; roll += step) {
		for (ssize_t pitch = min; pitch <= max; pitch += step) {
			for (ssize_t yaw = min; yaw <= max; yaw += step) {

				Eigen::Vector3d expected_deg(roll, pitch, yaw);
				Eigen::Vector3d expected = expected_deg * deg_to_rad;

				std::stringstream ss;

				ss << "DEG(" << expected_deg.x() << ", " << expected_deg.y() << ", " << expected_deg.z() << ")  ";
				ss << "RAD(" << expected.x() << ", " << expected.y() << ", " << expected.z() << ")";

				SCOPED_TRACE(ss.str());

				// rpy->q->rpy->q
				auto q1 = UAS::quaternion_from_rpy(expected);
				auto rpy = UAS::quaternion_to_rpy(q1);
				auto q2 = UAS::quaternion_from_rpy(rpy);

				// direct assumption is failed at ranges outside 0..pi
				//EXPECT_NEAR(expected.x(), rpy.x(), epsilon);
				//EXPECT_NEAR(expected.y(), rpy.y(), epsilon);
				//EXPECT_NEAR(expected.z(), rpy.z(), epsilon);

				// at -pi..0 we got complimentary q2 to q
				//EXPECT_QUATERNION(q1, q2, epsilon);

				// instead of direct comparision we rotate other quaternion and then compare results
				auto tq1 = q1 * test_orientation * q1.inverse();
				auto tq2 = q2 * test_orientation * q2.inverse();

				EXPECT_QUATERNION(tq1, tq2, epsilon);
			}
		}
	}
}

// UAS::quaternion_to_rpy() is not compatible with tf2::Matrix3x3(q).getRPY()

TEST(UAS, quaternion_get_yaw__123)
{
	// with pich >= pi/2 got incorrect result.
	// so 1, 2, 3 rad (57, 115, 172 deg) replaced with 60, 89, 172 deg
	auto q = UAS::quaternion_from_rpy(60.0 * deg_to_rad, 89.0 * deg_to_rad, 3.0);

	EXPECT_NEAR(3.0, UAS::quaternion_get_yaw(q), epsilon);
}

TEST(UAS, quaternion_get_yaw__pm_pi)
{
	// in degrees
	const ssize_t min = -180;
	const ssize_t max = 180;
	const ssize_t step = 1;

	for (ssize_t yaw = min; yaw <= max; yaw += step) {
		Eigen::Vector3d expected_deg(1.0, 2.0, yaw);
		Eigen::Vector3d expected = expected_deg * deg_to_rad;

		std::stringstream ss;

		ss << "DEG(" << expected_deg.x() << ", " << expected_deg.y() << ", " << expected_deg.z() << ")  ";
		ss << "RAD(" << expected.x() << ", " << expected.y() << ", " << expected.z() << ")";

		SCOPED_TRACE(ss.str());

		auto q1 = UAS::quaternion_from_rpy(expected);
		double q1_yaw = UAS::quaternion_get_yaw(q1);

		EXPECT_NEAR(expected.z(), q1_yaw, epsilon);
	}
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
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
