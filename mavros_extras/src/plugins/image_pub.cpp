/**
 * @brief Image pub plugin
 * @file image_pub.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace mavplugin {
namespace enc = sensor_msgs::image_encodings;

/**
 * @brief Image pub plugin
 *
 * This plugin recive image from mavlink stream and publish in ROS.
 */
class ImagePubPlugin : public MavRosPlugin {
public:
	ImagePubPlugin() :
		im_nh("~image"),
		im_width(0), im_height(0),
		im_size(0), im_packets(0), im_payload(0),
		im_seqnr(0), im_type(255),
		im_buffer {}
	{ };

	void initialize(UAS &uas_)
	{
		im_nh.param<std::string>("frame_id", frame_id, "px4flow");

		itp = boost::make_shared<image_transport::ImageTransport>(im_nh);
		image_pub = itp->advertise("camera_image", 1);
	}

	const message_map get_rx_handlers() {
		return {
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE, &ImagePubPlugin::handle_data_transmission_handshake),
			       MESSAGE_HANDLER(MAVLINK_MSG_ID_ENCAPSULATED_DATA, &ImagePubPlugin::handle_encapsulated_data)
		};
	}

private:
	ros::NodeHandle im_nh;

	boost::shared_ptr<image_transport::ImageTransport> itp;
	image_transport::Publisher image_pub;

	std::string frame_id;

	size_t im_width, im_height;
	size_t im_size, im_packets, im_payload;
	size_t im_seqnr;
	uint8_t im_type;
	std::vector<uint8_t> im_buffer;

	//! Maximum difference for im_buffer.capacity() and im_size (100 KiB)
	static constexpr size_t MAX_BUFFER_RESERVE_DIFF = 0x20000;

	bool check_supported_type(uint8_t type) const {
		return (type == MAVLINK_DATA_STREAM_IMG_JPEG ||
				type == MAVLINK_DATA_STREAM_IMG_BMP ||
				type == MAVLINK_DATA_STREAM_IMG_RAW8U ||
				/*type == MAVLINK_DATA_STREAM_IMG_RAW32U ||*/
				type == MAVLINK_DATA_STREAM_IMG_PGM ||
				type == MAVLINK_DATA_STREAM_IMG_PNG);
	}

	void publish_raw8u_image() {
		auto image = boost::make_shared<sensor_msgs::Image>();

		image->header.frame_id = frame_id;
		image->header.stamp = ros::Time::now();
		image->height = im_height;
		image->width = im_width;
		image->encoding = enc::MONO8;
		image->is_bigendian = false;
		image->step = im_width;
		image->data = im_buffer;

		image_pub.publish(image);
	}

	void publish_compressed_image() {
		cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

		cv_ptr->header.frame_id = frame_id;
		cv_ptr->header.stamp = ros::Time::now();

		try {
			cv_ptr->image = cv::imdecode(cv::Mat(im_buffer), CV_LOAD_IMAGE_UNCHANGED);
			if (cv_ptr->image.channels() == 1)
				cv_ptr->encoding = enc::MONO8;
			else
				cv_ptr->encoding = enc::BGR8;
		}
		catch (cv::Exception &ex) {
			ROS_ERROR_NAMED("image", "IMG: %s", ex.what());
			return;
		}

		image_pub.publish(cv_ptr->toImageMsg());
	}

	void publish_image() {
		switch (im_type) {
		case MAVLINK_DATA_STREAM_IMG_RAW8U:
			publish_raw8u_image();
			break;
		case MAVLINK_DATA_STREAM_IMG_JPEG:
		case MAVLINK_DATA_STREAM_IMG_BMP:
		case MAVLINK_DATA_STREAM_IMG_PGM:
		case MAVLINK_DATA_STREAM_IMG_PNG:
			publish_compressed_image();
			break;
		default:
			ROS_ERROR_NAMED("image", "IMG: Unsupported image type: %d", im_type);
		}
	}

	void handle_data_transmission_handshake(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		mavlink_data_transmission_handshake_t img_header;
		mavlink_msg_data_transmission_handshake_decode(msg, &img_header);

		if (!check_supported_type(img_header.type)) {
			ROS_WARN_NAMED("image", "IMG: Unknown stream type: %d", img_header.type);
			im_packets = 0;
			return;
		}

		// Note: no mutex, because all work done by one thread (reader)
		im_seqnr = 0;
		im_type = img_header.type;
		im_size = img_header.size;
		im_width = img_header.width;
		im_height = img_header.height;
		im_packets = img_header.packets;
		im_payload = img_header.payload;

		ROS_DEBUG_NAMED("image", "IMG: header: %zu x %zu t:%d, %zu bytes in %zu packets",
				im_width, im_height, im_type,
				im_size, im_packets);

		// prepare image buffer
		im_buffer.clear();
		if (im_buffer.capacity() < im_size ||
				im_buffer.capacity() > im_size + MAX_BUFFER_RESERVE_DIFF) {
			im_buffer.reserve(im_size);
		}
	}

	void handle_encapsulated_data(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		if (im_packets == 0)
			return;

		mavlink_encapsulated_data_t img_data;
		mavlink_msg_encapsulated_data_decode(msg, &img_data);

		size_t seqnr = img_data.seqnr;
		//ROS_DEBUG_NAMED("image", "IMG: chunk %2zu, waiting %2zu", seqnr, im_seqnr);

		if (seqnr + 1 > im_packets) {
			ROS_ERROR_NAMED("image", "IMG: More data packets, than specified in handshake, seqnr: %zu, packets: %zu",
					seqnr, im_packets);
			im_packets = 0;
			return;
		}

		if (seqnr > im_seqnr) {
			ROS_WARN_NAMED("image", "IMG: %zu data packets probably lost", seqnr - im_seqnr);
			// we lost some packets (or packet ordering changed by underlining transport (UDP)
			im_buffer.resize(std::min(im_size, (seqnr - 1) * im_payload), 0);
			im_seqnr = seqnr;
		}

		size_t bytes_to_copy = im_payload;
		if (seqnr * im_payload + bytes_to_copy >= im_size)
			bytes_to_copy = im_size - seqnr * im_payload;

		if (seqnr == im_seqnr) {
			// insert waiting packet
			im_seqnr++;
			im_buffer.insert(im_buffer.end(), img_data.data, img_data.data + bytes_to_copy);
		}
		else {
			// reordered packet arrives (seqnr < im_seqnr)
			ROS_DEBUG_NAMED("image", "IMG: reordered data message, seqnr: %zu, waiting: %zu",
					seqnr, im_seqnr);
			memcpy(im_buffer.data() + (seqnr * im_payload), img_data.data, bytes_to_copy);
		}

		if (seqnr + 1 == im_packets) {
			im_packets = 0;
			publish_image();
		}
	}
};
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::ImagePubPlugin, mavplugin::MavRosPlugin)
