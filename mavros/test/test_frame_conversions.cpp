/**
 * Test libmavros frame conversion utilities
 */

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <mavros/mavros_uas.h>

using mavros::UAS;

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



int main(int argc, char **argv)
{
	//ros::init(argc, argv, "mavros_test", ros::init_options::AnonymousName);
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
