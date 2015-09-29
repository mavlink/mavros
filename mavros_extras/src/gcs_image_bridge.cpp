/**
 * @brief MAVROS GCS proxy with Image sender
 * @file gcs_image_bridge.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 */
/*
 * Copyright 2014 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <ros/ros.h>
#include <ros/console.h>

#include <mavros/utils.h>
#include <mavros/mavlink_diag.h>
#include <mavconn/interface.h>

#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace mavros;
using namespace mavconn;
namespace enc = sensor_msgs::image_encodings;

int jpeg_quality;
MAVConnInterface::Ptr gcs_link;
ros::Publisher mavlink_pub;

void mavlink_pub_cb(const mavlink_message_t *mmsg, uint8_t sysid, uint8_t compid)
{
	auto rmsg = boost::make_shared<mavros_msgs::Mavlink>();

	rmsg->header.stamp = ros::Time::now();
	mavros_msgs::mavlink::convert(*mmsg, *rmsg);
	mavlink_pub.publish(rmsg);
}

void mavlink_sub_cb(const mavros_msgs::Mavlink::ConstPtr &rmsg)
{
	mavlink_message_t mmsg;

	if (mavros_msgs::mavlink::convert(*rmsg, mmsg))
		gcs_link->send_message(&mmsg, rmsg->sysid, rmsg->compid);
	else
		ROS_ERROR("Packet drop: illegal payload64 size");
}

void send_jpeg_image(std::vector<uint8_t> &jpeg_buffer, int jpeg_quality,
		int height, int width)
{
	constexpr size_t PAYLOAD_SIZE = sizeof(mavlink_encapsulated_data_t::data);
	mavlink_message_t msg;

	if (jpeg_buffer.empty()) {
		ROS_ERROR("IMG: Empty JPEG buffer!");
		return;
	}

	size_t packet_count = jpeg_buffer.size() / PAYLOAD_SIZE + 1;
	if (jpeg_buffer.capacity() < packet_count * PAYLOAD_SIZE) {
		// preventing copying unowned data in next step
		ROS_DEBUG("IMG: Reserved: %zu -> %zu bytes", jpeg_buffer.capacity(),
				packet_count * PAYLOAD_SIZE);
		jpeg_buffer.reserve(packet_count * PAYLOAD_SIZE);
	}

	ROS_DEBUG("IMG: Send image %d x %d, %zu bytes in %zu packets",
			width, height, jpeg_buffer.size(), packet_count);

	mavlink_msg_data_transmission_handshake_pack_chan(
		gcs_link->get_system_id(),
		gcs_link->get_component_id(),
		gcs_link->get_channel(),
		&msg,
		MAVLINK_DATA_STREAM_IMG_JPEG,
		jpeg_buffer.size(),
		width,
		height,
		packet_count,
		PAYLOAD_SIZE,
		jpeg_quality);
	gcs_link->send_message(&msg);

	for (size_t seqnr = 0; seqnr < packet_count; seqnr++) {
		mavlink_msg_encapsulated_data_pack_chan(
				gcs_link->get_system_id(),
				gcs_link->get_component_id(),
				gcs_link->get_channel(),
				&msg,
				seqnr,
				jpeg_buffer.data() + (PAYLOAD_SIZE * seqnr));
		gcs_link->send_message(&msg);

		//ROS_DEBUG("IMG: chunk %2zu, %p->%p", seqnr, jpeg_buffer.data(),
		//		jpeg_buffer.data() + (PAYLOAD_SIZE * seqnr));
	}
}

void image_cb(const sensor_msgs::Image::ConstPtr &img_msg)
{
	constexpr size_t PAYLOAD_SIZE = sizeof(mavlink_encapsulated_data_t::data);
	cv_bridge::CvImageConstPtr cv_ptr;

	try {
		if (enc::isColor(img_msg->encoding))
			cv_ptr = cv_bridge::toCvShare(img_msg, "bgr8");
		else
			cv_ptr = cv_bridge::toCvShare(img_msg);

		// code from image_transport_plugins compressed_publisher.cpp
		std::vector<int> params;
		std::vector<uint8_t> jpeg_buffer;
		params.resize(3, 0);
		// typical image size 60-70 packet
		jpeg_buffer.reserve(80 * PAYLOAD_SIZE);

		params[0] = CV_IMWRITE_JPEG_QUALITY;
		params[1] = jpeg_quality;

		if (cv::imencode(".jpg", cv_ptr->image, jpeg_buffer, params)) {
			float comp_ratio = (float)(cv_ptr->image.rows *
					cv_ptr->image.cols *
					cv_ptr->image.elemSize())
				/ jpeg_buffer.size();

			ROS_DEBUG("IMG: JPEG quality %d, ratio %f", jpeg_quality, comp_ratio);

			send_jpeg_image(jpeg_buffer, jpeg_quality,
					cv_ptr->image.rows,
					cv_ptr->image.cols);
		}
		else {
			ROS_ERROR("IMG: cv::imencode (jpeg) failed");
			return;
		}
	}
	catch (cv_bridge::Exception &ex) {
		ROS_ERROR("IMG: %s", ex.what());
	}
	catch (cv::Exception &ex) {
		ROS_ERROR("IMG: %s", ex.what());
	}
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "gcs_image_bridge");
	ros::NodeHandle priv_nh("~");
	ros::NodeHandle mavlink_nh("mavlink");
	ros::Subscriber mavlink_sub;
	diagnostic_updater::Updater updater;
	mavros::MavlinkDiag gcs_link_diag("GCS bridge");
	image_transport::ImageTransport it(mavlink_nh);
	image_transport::Subscriber image_sub;

	std::string gcs_url;
	priv_nh.param<std::string>("gcs_url", gcs_url, "udp://@");
	priv_nh.param("jpeg_quality", jpeg_quality, 60);

	try {
		gcs_link = MAVConnInterface::open_url(gcs_url);
		gcs_link_diag.set_mavconn(gcs_link);
		gcs_link_diag.set_connection_status(true);
	}
	catch (mavconn::DeviceError &ex) {
		ROS_FATAL("GCS: %s", ex.what());
		return 0;
	}

	mavlink_pub = mavlink_nh.advertise<mavros_msgs::Mavlink>("to", 10);
	gcs_link->message_received.connect(mavlink_pub_cb);

	mavlink_sub = mavlink_nh.subscribe("from", 10, mavlink_sub_cb,
			ros::TransportHints()
				.unreliable().maxDatagramSize(1024)
				.reliable());

	image_sub = it.subscribe("gcs_image", 1, image_cb);

	// setup updater
	updater.setHardwareID(gcs_url);
	updater.add(gcs_link_diag);

	// updater spinner
	auto diag_timer = priv_nh.createTimer(ros::Duration(1.0),
			[&](const ros::TimerEvent &evt) {
				updater.update();
			});
	diag_timer.start();

	ros::spin();
	return 0;
}

