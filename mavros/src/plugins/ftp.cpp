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

#include <std_srvs/Empty.h>
#include <mavros/FileEntry.h>
#include <mavros/FileList.h>
#include <mavros/FileOpen.h>
#include <mavros/FileClose.h>
#include <mavros/FileRead.h>

// enable debugging messages
#define FTP_LL_DEBUG

namespace mavplugin {

/**
 * @brief FTP Request message abstraction class
 *
 * @note This class not portable, and works on little-endian machines only.
 */
class FTPRequest {
public:
	/// @brief This is the payload which is in mavlink_file_transfer_protocol_t.payload.
	/// We pad the structure ourselves to 32 bit alignment to avoid usage of any pack pragmas.
	struct PayloadHeader {
		uint16_t	seqNumber;	///< sequence number for message
		uint8_t		session;	///< Session id for read and write commands
		uint8_t		opcode;		///< Command opcode
		uint8_t		size;		///< Size of data
		uint8_t		req_opcode;	///< Request opcode returned in kRspAck, kRspNak message
		uint8_t		padding[2];	///< 32 bit aligment padding
		uint32_t	offset;		///< Offsets for List and Read commands
		uint8_t		data[];		///< command data, varies by Opcode
	};

	/// @brief Command opcodes
	enum Opcode : uint8_t {
		kCmdNone,		///< ignored, always acked
		kCmdTerminateSession,	///< Terminates open Read session
		kCmdResetSessions,	///< Terminates all open Read sessions
		kCmdListDirectory,	///< List files in <path> from <offset>
		kCmdOpenFile,		///< Opens file at <path> for reading, returns <session>
		kCmdReadFile,		///< Reads <size> bytes from <offset> in <session>
		kCmdCreateFile,		///< Creates file at <path> for writing, returns <session>
		kCmdWriteFile,		///< Appends <size> bytes to file in <session>
		kCmdRemoveFile,		///< Remove file at <path>
		kCmdCreateDirectory,	///< Creates directory at <path>
		kCmdRemoveDirectory,	///< Removes Directory at <path>, must be empty

		kRspAck = 128,		///< Ack response
		kRspNak			///< Nak response
	};

	/// @brief Error codes returned in Nak response.
	enum ErrorCode : uint8_t {
		kErrNone,
		kErrFail,			///< Unknown failure
		kErrFailErrno,			///< Command failed, errno sent back in PayloadHeader.data[1]
		kErrInvalidDataSize,		///< PayloadHeader.size is invalid
		kErrInvalidSession,		///< Session is not currently open
		kErrNoSessionsAvailable,	///< All available Sessions in use
		kErrEOF,			///< Offset past end of file for List and Read commands
		kErrUnknownCommand		///< Unknown command opcode
	};

	static const char	DIRENT_FILE = 'F';
	static const char	DIRENT_DIR = 'D';
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

	uint32_t *data_u32() {
		return reinterpret_cast<uint32_t *>(header()->data);
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

#ifdef FTP_LL_DEBUG
		auto hdr = header();
		ROS_DEBUG_NAMED("ftp", "FTP:rm: SEQ(%u) SESS(%u) OPCODE(%u) RQOP(%u) SZ(%u) OFF(%u)",
				hdr->seqNumber, hdr->session, hdr->opcode, hdr->req_opcode, hdr->size, hdr->offset);
#endif

		return UAS_FCU(uas)->get_system_id() == message.target_system;
	}

	/**
	 * @brief Encode and send message
	 */
	void send(UAS *uas, uint16_t seqNumber) {
		mavlink_message_t msg;

		auto hdr = header();
		hdr->seqNumber = seqNumber;
		hdr->req_opcode = kCmdNone;

#ifdef FTP_LL_DEBUG
		ROS_DEBUG_NAMED("ftp", "FTP:sm: SEQ(%u) SESS(%u) OPCODE(%u) SZ(%u) OFF(%u)",
				hdr->seqNumber, hdr->session, hdr->opcode, hdr->size, hdr->offset);
#endif

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
		uas(nullptr),
		op_state(OP_IDLE),
		last_send_seqnr(0),
		active_session(0),
		is_error(false),
		list_offset(0),
		read_offset(0),
		open_size(0),
		read_size(0),
		read_buffer{}
	{ }

	void initialize(UAS &uas_,
			ros::NodeHandle &nh,
			diagnostic_updater::Updater &diag_updater)
	{
		uas = &uas_;

		ftp_nh = ros::NodeHandle(nh, "ftp");

		list_srv = ftp_nh.advertiseService("list", &FTPPlugin::list_cb, this);
		open_srv = ftp_nh.advertiseService("open", &FTPPlugin::open_cb, this);
		close_srv = ftp_nh.advertiseService("close", &FTPPlugin::close_cb, this);
		read_srv = ftp_nh.advertiseService("read", &FTPPlugin::read_cb, this);
		reset_srv = ftp_nh.advertiseService("reset", &FTPPlugin::reset_cb, this);
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
	ros::ServiceServer open_srv;
	ros::ServiceServer close_srv;
	ros::ServiceServer read_srv;
	ros::ServiceServer reset_srv;

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
	uint32_t active_session;

	std::mutex cond_mutex;
	std::condition_variable cond;	//!< wait condvar
	bool is_error;			//!< error signaling flag (timeout/proto error)

	// FTP:List
	uint32_t list_offset;
	std::string list_path;
	std::vector<mavros::FileEntry> list_entries;

	// FTP:Open / FTP:Close
	std::string open_path;
	size_t open_size;
	std::map<std::string, uint32_t> session_file_map;

	// FTP:Read
	size_t read_size;
	uint32_t read_offset;
	std::vector<uint8_t> read_buffer;

	// FTP:Write not implemented in FW

	// Timeouts
	static constexpr int LIST_TIMEOUT_MS = 15000;
	static constexpr int OPEN_TIMEOUT_MS = 500;
	static constexpr int CHUNK_TIMEOUT_MS = 500;

	//! Maximum difference between allocated space and used
	static constexpr size_t MAX_RESERVE_DIFF = 0x10000;

	//! @todo timeout timer
	//! @todo write support

	/* -*- message handler -*- */

	void handle_file_transfer_protocol(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
		FTPRequest req;
		if (!req.decode(uas, msg)) {
			ROS_DEBUG_NAMED("ftp", "FTP: Wrong System Id, MY %u, TGT %u",
					UAS_FCU(uas)->get_system_id(), req.get_target_system_id());
			return;
		}

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
		else {
			ROS_ERROR_NAMED("ftp", "FTP: Unknown request response: %u", req.header()->opcode);
			go_idle(true);
		}
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
		auto hdr = req.header();
		auto error_code = static_cast<FTPRequest::ErrorCode>(req.data()[0]);
		int req_errno = 0;
		OpState prev_op = op_state;

		ROS_ASSERT(hdr->size == 1 || (error_code == FTPRequest::kErrFailErrno && hdr->size == 2));

		op_state = OP_IDLE;
		if (error_code == FTPRequest::kErrFailErrno)
			req_errno = req.data()[1];

		if (prev_op == OP_LIST && error_code == FTPRequest::kErrEOF) {
			/* dir list done */
			list_directory_end();
			return;
		}
		else if (prev_op == OP_READ && error_code == FTPRequest::kErrEOF) {
			/* read done */
			read_file_end();
			return;
		}

		ROS_ERROR_NAMED("ftp", "FTP: NAck: %u Opcode: %u State: %u Errno: %d (%s)",
				error_code, hdr->req_opcode, prev_op, req_errno, strerror(req_errno));
		go_idle(true);
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
		auto hdr = req.header();

		ROS_DEBUG_NAMED("ftp", "FTP:m: ACK Open SZ(%u)", hdr->size);
		ROS_ASSERT(hdr->size == 0 || hdr->size == sizeof(uint32_t));
		if (hdr->size == sizeof(uint32_t)) {
			// kCmdOpenFile ACK
			open_size = *req.data_u32();
		}
		else if (hdr->size == 0) {
			// kCmdCreateFile ACK
			open_size = 0;
		}

		ROS_DEBUG_NAMED("ftp", "FTP:Open %s: success, session %u, size %zu",
				open_path.c_str(), hdr->session, open_size);
		session_file_map[open_path] = hdr->session;
		go_idle(false);
	}

	void handle_ack_read(FTPRequest &req) {
		auto hdr = req.header();

		ROS_DEBUG_NAMED("ftp", "FTP:m: ACK Read SZ(%u)", hdr->size);
		if (hdr->session != active_session) {
			ROS_ERROR_NAMED("ftp", "FTP:Read unexpected session");
			go_idle(true);
			return;
		}

		if (hdr->offset != read_offset) {
			ROS_ERROR_NAMED("ftp", "FTP:Read different offset");
			go_idle(true);
			return;
		}

		// kCmdReadFile return cunks of DATA_MAXSZ or smaller (last chunk)
		// We requested specific amount of data, that can be smaller,
		// but not larger.
		const size_t bytes_left = read_size - read_buffer.size();
		const size_t bytes_to_copy = std::min<size_t>(bytes_left, hdr->size);

		read_buffer.insert(read_buffer.end(), req.data(), req.data() + bytes_to_copy);
		//! @todo excancge speed calculation

		if (bytes_to_copy == FTPRequest::DATA_MAXSZ) {
			// Possibly more data
			read_offset += bytes_to_copy;
			send_read_command();
		}
		else
			read_file_end();
	}

	/* -*- send helpers -*- */

	void go_idle(bool is_error_) {
		op_state = OP_IDLE;
		is_error = is_error_;
		cond.notify_all();
	}

	void send_reset() {
		ROS_DEBUG_NAMED("ftp", "FTP:m: kCmdResetSessions");
		if (session_file_map.size() > 0) {
			ROS_WARN_NAMED("ftp", "FTP: Reset closes %zu sessons",
					session_file_map.size());
			session_file_map.clear();
		}

		op_state = OP_ACK;
		FTPRequest req(FTPRequest::kCmdResetSessions);
		req.send(uas, last_send_seqnr);
	}

	void send_list_command() {
		ROS_DEBUG_STREAM_NAMED("ftp", "FTP:m: kCmdListDirectory: " << list_path << " off: " << list_offset);
		FTPRequest req(FTPRequest::kCmdListDirectory);
		req.header()->offset = list_offset;
		req.set_data_string(list_path);
		req.send(uas, last_send_seqnr);
	}

	void send_open_command() {
		ROS_DEBUG_STREAM_NAMED("ftp", "FTP:m: kCmdOpenFile: " << open_path);
		FTPRequest req(FTPRequest::kCmdOpenFile);
		req.header()->offset = 0;
		req.set_data_string(open_path);
		req.send(uas, last_send_seqnr);
	}

	void send_create_command() {
		ROS_DEBUG_STREAM_NAMED("ftp", "FTP:m: kCmdCreateFile: " << open_path);
		FTPRequest req(FTPRequest::kCmdCreateFile);
		req.header()->offset = 0;
		req.set_data_string(open_path);
		req.send(uas, last_send_seqnr);
	}

	void send_terminate_command(uint32_t session) {
		ROS_DEBUG_STREAM_NAMED("ftp", "FTP:m: kCmdTerminateSession: " << session);
		FTPRequest req(FTPRequest::kCmdTerminateSession, session);
		req.header()->offset = 0;
		req.header()->size = 0;
		req.send(uas, last_send_seqnr);
	}

	void send_read_command() {
		// read operation always try read DATA_MAXSZ block (hdr->size ignored)
		ROS_DEBUG_STREAM_NAMED("ftp", "FTP:m: kCmdReadFile: " << active_session << " off: " << read_offset);
		FTPRequest req(FTPRequest::kCmdReadFile, active_session);
		req.header()->offset = read_offset;
		req.header()->size = 0 /* FTPRequest::DATA_MAXSZ */;
		req.send(uas, last_send_seqnr);
	}

	/* how to open existing file to write? */

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

		// skip adding special files (now it skipped at server)
		//if (ent.name == "." || ent.name == "..")
		//	return;

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

	bool open_file(std::string path, int mode) {
		open_path = path;
		open_size = 0;
		op_state = OP_OPEN;

		if (mode == mavros::FileOpenRequest::MODE_READ)
			send_open_command();
		//else if (mode == mavros::FileOpenRequest::MODE_WRITE)
		//	send_create_command();
		else {
			op_state = OP_IDLE;
			return false;
		}

		return true;
	}

	bool close_file(std::string path) {
		auto it = session_file_map.find(path);
		if (it == session_file_map.end()) {
			ROS_ERROR_NAMED("ftp", "FTP:Close %s: not opened", path.c_str());
			return false;
		}

		op_state = OP_ACK;
		send_terminate_command(it->second);
		session_file_map.erase(it);
		return true;
	}

	void read_file_end() {
		ROS_DEBUG_NAMED("ftp", "FTP:Read done");
		go_idle(false);
	}

	bool read_file(std::string path, size_t off, size_t len) {
		auto it = session_file_map.find(path);
		if (it == session_file_map.end()) {
			ROS_ERROR_NAMED("ftp", "FTP:Read %s: not opened", path.c_str());
			return false;
		}

		op_state = OP_READ;
		active_session = it->second;
		read_size = len;
		read_offset = off;
		read_buffer.clear();
		if (read_buffer.capacity() < len ||
				read_buffer.capacity() > len + MAX_RESERVE_DIFF) {
			// reserve memory
			read_buffer.reserve(len);
		}

		send_read_command();
		return true;
	}

	static constexpr int read_compute_timeout(size_t len) {
		return CHUNK_TIMEOUT_MS * (len + 1) / FTPRequest::DATA_MAXSZ;
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

	bool open_cb(mavros::FileOpen::Request &req,
			mavros::FileOpen::Response &res) {
		if (op_state != OP_IDLE) {
			ROS_ERROR_NAMED("ftp", "FTP: Busy");
			return false;
		}

		// only one session per file
		auto it = session_file_map.find(req.file_path);
		if (it != session_file_map.end()) {
			ROS_ERROR_NAMED("ftp", "FTP: File %s: already opened",
					req.file_path.c_str());
			return false;
		}

		res.success = open_file(req.file_path, req.mode);
		if (res.success)
			res.success = wait_completion(OPEN_TIMEOUT_MS);
		if (res.success)
			res.size = open_size;

		//! @todo return system error code for fuse driver
		//! @todo inactivity close timer

		return true;
	}

	bool close_cb(mavros::FileClose::Request &req,
			mavros::FileClose::Response &res) {
		if (op_state != OP_IDLE) {
			ROS_ERROR_NAMED("ftp", "FTP: Busy");
			return false;
		}

		res.success = close_file(req.file_path);
		if (res.success)
			res.success = wait_completion(OPEN_TIMEOUT_MS);

		return true;
	}

	bool read_cb(mavros::FileRead::Request &req,
			mavros::FileRead::Response &res) {
		if (op_state != OP_IDLE) {
			ROS_ERROR_NAMED("ftp", "FTP: Busy");
			return false;
		}

		res.success = read_file(req.file_path, req.offset, req.size);
		if (res.success)
			res.success = wait_completion(read_compute_timeout(req.size));
		if (res.success)
			res.data = read_buffer;

		return true;
	}

	/**
	 * @brief Reset communication on both sides.
	 * @note This call break other calls, so use carefully.
	 */
	bool reset_cb(std_srvs::Empty::Request &req,
			std_srvs::Empty::Response &res) {
		send_reset();
		return true;
	}
};

}; // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::FTPPlugin, mavplugin::MavRosPlugin)

