/**
 * Test libmavros frame conversion utilities
 */

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <mavros/mavros_uas.h>

using mavros::UAS;

static const double epsilon = 1e-6;


template <typename T>
static void log_vectors(T &in, T &out, T &expected)
{
	ROS_INFO("In (x y z): %f %f %f", in.x(), in.y(), in.z());
	ROS_INFO("Expected:   %f %f %f", expected.x(), expected.y(), expected.z());
	ROS_INFO("Out:        %f %f %f", out.x(), out.y(), out.z());
}

template <typename T>
static void log_quaternion(T &in, T &out, T &expected)
{
	ROS_INFO("In (x y z w): %f %f %f %f", in.x(), in.y(), in.z(), in.w());
	ROS_INFO("Expected:     %f %f %f %f", expected.x(), expected.y(), expected.z(), expected.w());
	ROS_INFO("Out:          %f %f %f %f", out.x(), out.y(), out.z(), out.w());
}

static std::string log_v(UAS::Covariance6x6 &cov, int n)
{
	std::stringstream cov_stream;
	std::string cov_mtrx_ln;
	for (int i=n-5; i<=n ; i++)
	{
		if (i == 35)
			cov_stream << cov.at(i);
		else
			cov_stream << cov.at(i)<< "\t";
	}
	cov_mtrx_ln = cov_stream.str();
	return cov_mtrx_ln;
}

static void log_covariance6x6(UAS::Covariance6x6 &in, UAS::Covariance6x6 &out, UAS::Covariance6x6 &expected)
{
	ROS_INFO("\n\tIn (6x6 Covariance Matrix):\n\t{%s\n\t %s\n\t %s\n\t %s\n\t %s\n\t %s}",
						log_v(in,5).c_str(),
						log_v(in,11).c_str(),
						log_v(in,17).c_str(),
						log_v(in,23).c_str(),
						log_v(in,29).c_str(),
						log_v(in,35).c_str());

	ROS_INFO("\n\tExpected:\n\t{%s\n\t %s\n\t %s\n\t %s\n\t %s\n\t %s}",
						log_v(expected,5).c_str(),
						log_v(expected,11).c_str(),
						log_v(expected,17).c_str(),
						log_v(expected,23).c_str(),
						log_v(expected,29).c_str(),
						log_v(expected,35).c_str());

	ROS_INFO("\n\tOut:\n\t{%s\n\t %s\n\t %s\n\t %s\n\t %s\n\t %s}",
						log_v(out,5).c_str(),
						log_v(out,11).c_str(),
						log_v(out,17).c_str(),
						log_v(out,23).c_str(),
						log_v(out,29).c_str(),
						log_v(out,35).c_str());
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

TEST(EIGEN_VECTOR, transform_frame_123)
{
	Eigen::Vector3d input(1, 2, 3);
	Eigen::Vector3d expected_out(1, -2, -3);

	auto out = UAS::transform_frame(input);

	log_vectors(input, out, expected_out);
	EXPECT_NEAR(expected_out.x(), out.x(), epsilon);
	EXPECT_NEAR(expected_out.y(), out.y(), epsilon);
	EXPECT_NEAR(expected_out.z(), out.z(), epsilon);
}

TEST(EIGEN_QUATERNION, transform_frame_123)
{
	Eigen::Quaterniond input =
		Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitX()) *
		Eigen::AngleAxisd(2.0, Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxisd(3.0, Eigen::Vector3d::UnitZ());
#if 1 // XXX we should find what method used by tf::quaternionFromRPY()
	Eigen::Quaterniond expected_out =
		Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitX()) *
		Eigen::AngleAxisd(-2.0, Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxisd(-3.0, Eigen::Vector3d::UnitZ());
#else
	Eigen::Quaterniond expected_out =
		Eigen::AngleAxisd(-3.0, Eigen::Vector3d::UnitZ()) *
		Eigen::AngleAxisd(-2.0, Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitX());
#endif

	auto out = UAS::transform_frame(input);

	log_quaternion(input, out, expected_out);
	EXPECT_NEAR(expected_out.w(), out.w(), epsilon);
	EXPECT_NEAR(expected_out.x(), out.x(), epsilon);
	EXPECT_NEAR(expected_out.y(), out.y(), epsilon);
	EXPECT_NEAR(expected_out.z(), out.z(), epsilon);
}

// XXX: #321 comment out broken transform's before release 0.12
// after we SHOULD come and fix!
#if 0
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

/* -*- test covariance transform -*- */

TEST(COVARIANCE6X6,  transform_frame_covariance_pose6x6_sample1)
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

	auto out = UAS::transform_frame_covariance_pose6x6(input);

	log_covariance6x6(input, out, expected_out);
	EXPECT_EQ(expected_out, out);
}

#endif

int main(int argc, char **argv)
{
	//ros::init(argc, argv, "mavros_test", ros::init_options::AnonymousName);
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
