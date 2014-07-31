/**
 * @brief Image pub plugin
 * @file image_pub.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <cv.h>
#include <highgui.h>

namespace enc = sensor_msgs::image_encodings;

namespace mavplugin {

class ImagePubPlugin : public MavRosPlugin {
public:
	ImagePubPlugin()
	{ };

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		waiting_data = false;
		itp = boost::make_shared<image_transport::ImageTransport>(nh);
		image_pub = itp->advertise("camera_image", 1);
	}

	const std::string get_name() const {
		return "ImagePub";
	}

	const std::vector<uint8_t> get_supported_messages() const {
		return {
			MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE,
			MAVLINK_MSG_ID_ENCAPSULATED_DATA
		};
	}

	void message_rx_cb(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		switch (msg->msgid) {
		case MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE:
			handle_data_transmission_handshake(msg);
			break;

		case MAVLINK_MSG_ID_ENCAPSULATED_DATA:
			handle_encapsulated_data(msg);
			break;
		}
	}

private:
	boost::shared_ptr<image_transport::ImageTransport> itp;
	image_transport::Publisher image_pub;

	bool waiting_data;
	size_t im_width, im_height;
	size_t im_size, im_packets, im_payload;
	size_t im_seqnr;
	uint8_t im_type;
	std::vector<uint8_t> im_buffer;

	static constexpr size_t MAX_BUFFER_RESERVE_DIFF = 0x10000;

	bool check_supported_type(uint8_t type) {
		return (type == MAVLINK_DATA_STREAM_IMG_JPEG ||
				type == MAVLINK_DATA_STREAM_IMG_BMP ||
				type == MAVLINK_DATA_STREAM_IMG_RAW8U ||
				type == MAVLINK_DATA_STREAM_IMG_RAW32U ||
				type == MAVLINK_DATA_STREAM_IMG_PGM ||
				type == MAVLINK_DATA_STREAM_IMG_PNG);
	}

	void publish_image() {
		ROS_WARN("data available: %zu, %zu", im_size, im_buffer.size());
	}

	void handle_data_transmission_handshake(const mavlink_message_t *msg) {
		mavlink_data_transmission_handshake_t img_header;
		mavlink_msg_data_transmission_handshake_decode(msg, &img_header);

		if (!check_supported_type(img_header.type)) {
			ROS_WARN_NAMED("image", "IMG: Unknown stream type: %d", img_header.type);
			waiting_data = false;
			return;
		}

		// Note: no mutex, because all work done by one thread (reader)
		waiting_data = true;
		im_seqnr = 0;
		im_type = img_header.type;
		im_size = img_header.size;
		im_width = img_header.width;
		im_height = img_header.height;
		im_packets = img_header.packets;
		im_payload = img_header.payload;

		// prepare image buffer
		im_buffer.clear();
		if (im_buffer.capacity() < im_size ||
				im_buffer.capacity() > im_size + MAX_BUFFER_RESERVE_DIFF) {
			im_buffer.reserve(im_size);
		}
	}

	void handle_encapsulated_data(const mavlink_message_t *msg) {
		if (!waiting_data)
			return;

		mavlink_encapsulated_data_t img_data;
		mavlink_msg_encapsulated_data_decode(msg, &img_data);

		size_t seqnr = img_data.seqnr;
		if (seqnr != im_seqnr) {
			// TODO: fill lost data by zeroes, and try to decode
			ROS_ERROR_NAMED("image", "IMG: Lost sync, seq: %zu, waiting: %zu", seqnr, im_seqnr);
			waiting_data = false;
			return;
		}

		if (seqnr + 1 > im_packets) {
			ROS_DEBUG_NAMED("image", "IMG: More data packets, than specified in handshake, seq: %zu", seqnr);
			waiting_data = false;
			return;
		}

		size_t bytes_to_copy = im_payload;
		if (seqnr * im_payload + bytes_to_copy >= im_size)
			bytes_to_copy = im_size - seqnr * im_payload;

		im_seqnr++;
		im_buffer.insert(im_buffer.end(), img_data.data, img_data.data + bytes_to_copy);

		if (seqnr + 1 == im_payload) {
			waiting_data = false;
			publish_image();
		}
	}
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::ImagePubPlugin, mavplugin::MavRosPlugin)
