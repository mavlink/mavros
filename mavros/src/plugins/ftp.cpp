/**
 * @brief FTP plugin
 * @file ftp.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014 Vladimir Ermakov.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <chrono>
#include <condition_variable>
#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <mavros/FileEntry.h>
#include <mavros/FileList.h>

namespace mavplugin {

// XXX: crc32() copy from NuttX lib_crc32.c
static const uint32_t crc32_tab[] =
{
  0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3,
  0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988, 0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91,
  0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
  0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5,
  0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172, 0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,
  0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
  0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f,
  0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924, 0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,
  0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
  0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01,
  0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e, 0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457,
  0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
  0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb,
  0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0, 0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9,
  0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
  0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81, 0xb7bd5c3b, 0xc0ba6cad,
  0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a, 0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683,
  0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
  0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7,
  0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5,
  0xd6d6a3e8, 0xa1d1937e, 0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
  0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79,
  0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236, 0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f,
  0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
  0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713,
  0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38, 0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21,
  0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
  0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45,
  0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db,
  0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
  0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf,
  0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94, 0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

/************************************************************************************************
 * Public Functions
 ************************************************************************************************/
/************************************************************************************************
 * Name: crc32part
 *
 * Description:
 *   Continue CRC calculation on a part of the buffer.
 *
 ************************************************************************************************/

uint32_t crc32part(const uint8_t *src, size_t len, uint32_t crc32val)
{
  size_t i;

  for (i = 0;  i < len;  i++)
    {
      crc32val = crc32_tab[(crc32val ^ src[i]) & 0xff] ^ (crc32val >> 8);
    }
  return crc32val;
}
// XXX: end


class FTPRequest {
public:
	/// @brief This is the payload which is in mavlink_file_transfer_protocol_t.payload.
	/// We pad the structure ourselves to 32 bit alignment to avoid usage of any pack pragmas.
	struct PayloadHeader {
		uint16_t	seqNumber;	///< sequence number for message
		uint8_t		session;	///< Session id for read and write commands
		uint8_t		opcode;		///< Command opcode
		uint8_t		size;		///< Size of data
		uint8_t		padding[3];	///< 32 bit aligment padding
		uint32_t	crc32;		///< CRC for entire Request structure, with crc32 and padding set to 0
		uint32_t	offset;		///< Offsets for List and Read commands
		uint8_t		data[];		///< command data, varies by Opcode
	};

	/// @brief Command opcodes
	enum Opcode : uint8_t {
		kCmdNone,	///< ignored, always acked
		kCmdTerminate,	///< Terminates open Read session
		kCmdReset,	///< Terminates all open Read sessions
		kCmdList,	///< List files in <path> from <offset>
		kCmdOpen,	///< Opens <path> for reading, returns <session>
		kCmdRead,	///< Reads <size> bytes from <offset> in <session>
		kCmdCreate,	///< Creates <path> for writing, returns <session>
		kCmdWrite,	///< Appends <size> bytes at <offset> in <session>
		kCmdRemove,	///< Remove file

		kRspAck,	///< Ack response
		kRspNak		///< Nak response
	};

	/// @brief Error codes returned in Nak response.
	enum ErrorCode : uint8_t {
		kErrNone,
		kErrNoRequest,
		kErrNoSession,
		kErrSequence,
		kErrNotDir,
		kErrNotFile,
		kErrEOF,
		kErrNotAppend,
		kErrTooBig,
		kErrIO,
		kErrPerm,
		kErrUnknownCommand,
		kErrCrc
        };

	static const char	DIRENT_FILE = 'F';
	static const char	DIRENT_DIR = 'D';
	static const char	DIRENT_UNKNOWN = 'U';
	static const uint8_t	DATA_MAXSZ = MAVLINK_MSG_FILE_TRANSFER_PROTOCOL_FIELD_PAYLOAD_LEN - sizeof(PayloadHeader);

	uint8_t *raw_payload() {
		return message.payload;
	}

	inline PayloadHeader *header() {
		return reinterpret_cast<PayloadHeader *>(message.payload);
	}

	uint8_t *data() {
		return header()->data;
	}

	char *data_c() {
		return reinterpret_cast<char *>(header()->data);
	}

	char *data_c_str() {

		// force null-termination
		if (header()->size < DATA_MAXSZ)
			data()[header()->size] = '\0';
		else
			data()[DATA_MAXSZ - 1] = '\0';

		return data_c();
	}

	void set_data_string(std::string &s) {
		strncpy(data_c(), s.c_str(), DATA_MAXSZ - 1);
		header()->size = strnlen(data_c(), DATA_MAXSZ);
	}

	uint8_t get_target_system_id() {
		return message.target_system;
	}

	/**
	 * @brief Decode and check target system
	 */
	bool decode(UAS *uas, const mavlink_message_t *msg) {
		mavlink_msg_file_transfer_protocol_decode(msg, &message);

		// XXX: crc debug
		auto hdr = header();
		uint32_t mcrc = hdr->crc32;
		hdr->crc32 = 0;
		hdr->padding[0] = 0;
		hdr->padding[1] = 0;
		hdr->padding[2] = 0;
		hdr->crc32 = crc32part(raw_payload(), hdr->size + sizeof(PayloadHeader), 0);

		ROS_DEBUG_NAMED("ftp", "FTP:rm: SEQ(%u) SESS(%u) OPCODE(%u) SZ(%u) OFF(%u) CRC(0x%08x : 0x%08x)",
				hdr->seqNumber, hdr->session, hdr->opcode, hdr->size, hdr->offset,
				mcrc, hdr->crc32);
		// XXX: end

		return UAS_FCU(uas)->get_system_id() == message.target_system;
	}

	/**
	 * @brief Encode and send message
	 */
	void send(UAS *uas, uint16_t seqNumber) {
		mavlink_message_t msg;

		// XXX: crc calc
		auto hdr = header();
		hdr->seqNumber = seqNumber;
		hdr->crc32 = 0;
		hdr->padding[0] = 0;
		hdr->padding[1] = 0;
		hdr->padding[2] = 0;
		hdr->crc32 = crc32part(raw_payload(), hdr->size + sizeof(PayloadHeader), 0);

		ROS_DEBUG_NAMED("ftp", "FTP:sm: SEQ(%u) SESS(%u) OPCODE(%u) SZ(%u) OFF(%u) CRC(0x%08x)",
				hdr->seqNumber, hdr->session, hdr->opcode, hdr->size, hdr->offset, hdr->crc32);
		// XXX: end

		mavlink_msg_file_transfer_protocol_pack_chan(UAS_PACK_CHAN(uas), &msg,
				0, // target_network
				UAS_PACK_TGT(uas),
				raw_payload());
		UAS_FCU(uas)->send_message(&msg);
	}

	FTPRequest() :
		message{}
	{ }

	explicit FTPRequest(Opcode op, uint8_t session = 0) :
		message{}
	{
		header()->session = session;
		header()->opcode = op;
	}

private:
	mavlink_file_transfer_protocol_t message;
};


/**
 * @brief FTP plugin.
 */
class FTPPlugin : public MavRosPlugin {
public:
	FTPPlugin() :
		op_state(OP_IDLE)
	{ }

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		uas = &uas_;

		ftp_nh = ros::NodeHandle(nh, "ftp");

		list_srv = ftp_nh.advertiseService("list", &FTPPlugin::list_cb, this);
	}

	std::string const get_name() const {
		return "FTP";
	}

	const message_map get_rx_handlers() {
		return {
			MESSAGE_HANDLER(MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL, &FTPPlugin::handle_file_transfer_protocol),
		};
	}

private:
	UAS *uas;
	ros::NodeHandle ftp_nh;
	ros::ServiceServer list_srv;

	enum OpState {
		OP_IDLE,
		OP_ACK,
		OP_LIST,
		OP_OPEN,
		OP_READ
		// TODO other functions
	};

	OpState op_state;
	uint16_t last_send_seqnr;

	std::mutex cond_mutex;
	std::condition_variable cond;	//!< wait condvar
	bool is_error;			//!< error signaling flag (timeout/proto error)

	// FTP:List
	uint32_t list_offset;
	std::string list_path;
	std::vector<mavros::FileEntry> list_entries;

	static constexpr int LIST_TIMEOUT_MS = 15000;

	/* -*- message handler -*- */

	void handle_file_transfer_protocol(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		FTPRequest req;
		if (!req.decode(uas, msg)) {
			ROS_DEBUG_NAMED("ftp", "FTP: Wrong System Id, MY %u, TGT %u",
					UAS_FCU(uas)->get_system_id(), req.get_target_system_id());
			return;
		}

		// Note crc32 check skipped

		const uint16_t incoming_seqnr = req.header()->seqNumber;
		const uint16_t expected_seqnr = last_send_seqnr + 1;
		if (incoming_seqnr != expected_seqnr) {
			ROS_WARN_NAMED("ftp", "FTP: Lost sync! seqnr: %u != %u",
					incoming_seqnr, expected_seqnr);
			go_idle(true);
			return;
		}

		last_send_seqnr = incoming_seqnr;

		// logic from QGCUASFileManager.cc
		if (req.header()->opcode == FTPRequest::kRspAck)
			handle_req_ack(req);
		else if (req.header()->opcode == FTPRequest::kRspNak)
			handle_req_nack(req);
		else
			ROS_ERROR_NAMED("ftp", "FTP: Unknown request response: %u", req.header()->opcode);
	}

	void handle_req_ack(FTPRequest &req) {
		switch (op_state) {
		case OP_IDLE: send_reset();		break;
		case OP_ACK:  go_idle(false);		break;
		case OP_LIST: handle_ack_list(req);	break;
		case OP_OPEN: handle_ack_open(req);	break;
		case OP_READ: handle_ack_read(req);	break;
		default:
			ROS_ERROR_NAMED("ftp", "FTP: wrong op_state");
			go_idle(true);
		}
	}

	void handle_req_nack(FTPRequest &req) {
		ROS_ASSERT(req.header()->size == 1);

		uint8_t error = req.data()[0];
		OpState prev_op = op_state;

		op_state = OP_IDLE;

		if (prev_op == OP_LIST && error == FTPRequest::kErrEOF) {
			/* dir list done */
			list_directory_end();
			return;
		}
		else if (prev_op == OP_READ && error == FTPRequest::kErrEOF) {
			/* read done */
		}

		ROS_ERROR_NAMED("ftp", "FTP: NAck: %u", error);
	}

	void handle_ack_list(FTPRequest &req) {
		auto hdr = req.header();

		ROS_DEBUG_NAMED("ftp", "FTP:m: ACK List SZ(%u) OFF(%u)", hdr->size, hdr->offset);
		if (hdr->offset != list_offset) {
			ROS_ERROR_NAMED("ftp", "FTP: Wring list offset, req %u, ret %u",
					list_offset, hdr->offset);
			go_idle(true);
			return;
		}

		uint8_t off = 0;
		uint32_t n_list_entries = 0;

		while (off < hdr->size) {
			const char *ptr = req.data_c() + off;
			const size_t bytes_left = hdr->size - off;

			size_t slen = strnlen(ptr, bytes_left);
			if (slen < 2) {
				ROS_ERROR_NAMED("ftp", "FTP: Incorrect list entry: %s", ptr);
				go_idle(true);
				return;
			}
			else if (slen == bytes_left) {
				ROS_ERROR_NAMED("ftp", "FTP: Missing NULL termination in list entry");
				go_idle(true);
				return;
			}

			if (ptr[0] == FTPRequest::DIRENT_FILE ||
					ptr[0] == FTPRequest::DIRENT_DIR) {
				add_dirent(ptr, slen);
			}
			else {
				ROS_WARN_NAMED("ftp", "FTP: Unknown list entry: %s", ptr);
			}

			off += slen + 1;
			n_list_entries++;
		}

		if (hdr->size == 0) {
			// dir empty, we are done
			list_directory_end();
		}
		else {
			ROS_ASSERT_MSG(n_list_entries > 0, "FTP:List don't parse entries");
			// Possibly more to come, try get more
			list_offset += n_list_entries;
			send_list_command();
		}
	}

	void handle_ack_open(FTPRequest &req) {
		ROS_WARN_NAMED("ftp", "FTP: open ack");
	}

	void handle_ack_read(FTPRequest &req) {
		ROS_WARN_NAMED("ftp", "FTP: read ack");
	}

	/* -*- send helpers -*- */

	void go_idle(bool is_error_) {
		op_state = OP_IDLE;
		is_error = is_error_;
		cond.notify_all();
	}

	void send_reset() {
		ROS_DEBUG_NAMED("ftp", "FTP:m: kCmdReset");
		op_state = OP_ACK;
		FTPRequest req(FTPRequest::kCmdReset);
		req.send(uas, last_send_seqnr);
	}

	void send_list_command() {
		ROS_DEBUG_STREAM_NAMED("ftp", "FTP:m: kCmdList: " << list_path << " off: " << list_offset);
		FTPRequest req(FTPRequest::kCmdList);
		req.header()->offset = list_offset;
		req.set_data_string(list_path);
		req.send(uas, last_send_seqnr);
	}

	/* -*- helpers -*- */

	void add_dirent(const char *ptr, size_t slen) {
		mavros::FileEntry ent;
		ent.size = 0;

		if (ptr[0] == FTPRequest::DIRENT_DIR) {
			ent.name.assign(ptr + 1, slen - 1);
			ent.type = mavros::FileEntry::TYPE_DIRECTORY;

			ROS_DEBUG_STREAM_NAMED("ftp", "FTP:List Dir: " << ent.name);
		}
		else {
			// ptr[0] == FTPRequest::DIRENT_FILE
			std::string name_size(ptr + 1, slen - 1);

			auto sep_it = std::find(name_size.begin(), name_size.end(), '\t');
			ent.name.assign(name_size.begin(), sep_it);
			ent.type = mavros::FileEntry::TYPE_FILE;

			if (sep_it != name_size.end()) {
				name_size.erase(name_size.begin(), sep_it + 1);
				if (name_size.size() != 0)
					ent.size = std::stoi(name_size);
			}

			ROS_DEBUG_STREAM_NAMED("ftp", "FTP:List File: " << ent.name << " SZ: " << ent.size);
		}

		// skip adding special files
		if (ent.name == "." || ent.name == "..")
			return;

		list_entries.push_back(ent);
	}

	void list_directory_end() {
		ROS_DEBUG_NAMED("ftp", "FTP:List done");
		go_idle(false);
	}

	void list_directory(std::string path) {

		list_offset = 0;
		list_path = path;
		list_entries.clear();
		op_state = OP_LIST;

		send_list_command();
	}

	bool wait_completion(const int msecs) {
		std::unique_lock<std::mutex> lock(cond_mutex);

		return cond.wait_for(lock, std::chrono::milliseconds(msecs))
			== std::cv_status::no_timeout
			&& !is_error;
	}

	/* -*- service callbacks -*- */

	bool list_cb(mavros::FileList::Request &req,
			mavros::FileList::Response &res) {

		if (op_state != OP_IDLE) {
			ROS_ERROR_NAMED("ftp", "FTP: Busy");
			return false;
		}

		list_directory(req.dir_path);
		res.success = wait_completion(LIST_TIMEOUT_MS);
		if (res.success)
			res.list = list_entries;

		return true;
	}
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::FTPPlugin, mavplugin::MavRosPlugin)

