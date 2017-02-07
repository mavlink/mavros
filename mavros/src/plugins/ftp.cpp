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
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <chrono>
#include <cerrno>
#include <condition_variable>
#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <std_srvs/Empty.h>
#include <mavros_msgs/FileEntry.h>
#include <mavros_msgs/FileList.h>
#include <mavros_msgs/FileOpen.h>
#include <mavros_msgs/FileClose.h>
#include <mavros_msgs/FileRead.h>
#include <mavros_msgs/FileWrite.h>
#include <mavros_msgs/FileRemove.h>
#include <mavros_msgs/FileMakeDir.h>
#include <mavros_msgs/FileRemoveDir.h>
#include <mavros_msgs/FileTruncate.h>
#include <mavros_msgs/FileRename.h>
#include <mavros_msgs/FileChecksum.h>

// enable debugging messages
//#define FTP_LL_DEBUG

#ifdef __APPLE__
#define EBADE 50   /* Invalid exchange */
#define EBADFD 81  /* File descriptor in bad state */
#define EBADRQC 54 /* Invalid request code */
#define EBADSLT 55 /* Invalid slot */
#endif

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
		kCmdOpenFileRO,		///< Opens file at <path> for reading, returns <session>
		kCmdReadFile,		///< Reads <size> bytes from <offset> in <session>
		kCmdCreateFile,		///< Creates file at <path> for writing, returns <session>
		kCmdWriteFile,		///< Writes <size> bytes to <offset> in <session>
		kCmdRemoveFile,		///< Remove file at <path>
		kCmdCreateDirectory,	///< Creates directory at <path>
		kCmdRemoveDirectory,	///< Removes Directory at <path>, must be empty
		kCmdOpenFileWO,		///< Opens file at <path> for writing, returns <session>
		kCmdTruncateFile,	///< Truncate file at <path> to <offset> length
		kCmdRename,		///< Rename <path1> to <path2>
		kCmdCalcFileCRC32,	///< Calculate CRC32 for file at <path>
		kCmdBurstReadFile,	///< Burst download session file

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
		kErrUnknownCommand,		///< Unknown command opcode
		kErrFailFileExists,		///< File exists already
		kErrFailFileProtected		///< File is write protected
	};

	static const char	DIRENT_FILE = 'F';
	static const char	DIRENT_DIR = 'D';
	static const char	DIRENT_SKIP = 'S';
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

	/**
	 * @brief Copy string to payload
	 *
	 * @param[in] s  payload string
	 * @note this function allow null termination inside string
	 *       it used to send multiple strings in one message
	 */
	void set_data_string(std::string &s) {
		size_t sz = (s.size() < DATA_MAXSZ - 1) ? s.size() : DATA_MAXSZ - 1;

		memcpy(data_c(), s.c_str(), sz);
		data_c()[sz] = '\0';
		header()->size = sz;
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
				0,	// target_network
				UAS_PACK_TGT(uas),
				raw_payload());
		UAS_FCU(uas)->send_message(&msg);
	}

	FTPRequest() :
		message {}
	{ }

	explicit FTPRequest(Opcode op, uint8_t session = 0) :
		message {}
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
		ftp_nh("~ftp"),
		uas(nullptr),
		op_state(OP_IDLE),
		last_send_seqnr(0),
		active_session(0),
		is_error(false),
		r_errno(0),
		list_offset(0),
		read_offset(0),
		write_offset(0),
		open_size(0),
		read_size(0),
		read_buffer {},
		checksum_crc32(0)
	{ }

	void initialize(UAS &uas_)
	{
		uas = &uas_;

		list_srv = ftp_nh.advertiseService("list", &FTPPlugin::list_cb, this);
		open_srv = ftp_nh.advertiseService("open", &FTPPlugin::open_cb, this);
		close_srv = ftp_nh.advertiseService("close", &FTPPlugin::close_cb, this);
		read_srv = ftp_nh.advertiseService("read", &FTPPlugin::read_cb, this);
		write_srv = ftp_nh.advertiseService("write", &FTPPlugin::write_cb, this);
		mkdir_srv = ftp_nh.advertiseService("mkdir", &FTPPlugin::mkdir_cb, this);
		rmdir_srv = ftp_nh.advertiseService("rmdir", &FTPPlugin::rmdir_cb, this);
		remove_srv = ftp_nh.advertiseService("remove", &FTPPlugin::remove_cb, this);
		truncate_srv = ftp_nh.advertiseService("truncate", &FTPPlugin::truncate_cb, this);
		reset_srv = ftp_nh.advertiseService("reset", &FTPPlugin::reset_cb, this);
		rename_srv = ftp_nh.advertiseService("rename", &FTPPlugin::rename_cb, this);
		checksum_srv = ftp_nh.advertiseService("checksum", &FTPPlugin::checksum_cb, this);
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
	ros::ServiceServer write_srv;
	ros::ServiceServer mkdir_srv;
	ros::ServiceServer rmdir_srv;
	ros::ServiceServer remove_srv;
	ros::ServiceServer rename_srv;
	ros::ServiceServer truncate_srv;
	ros::ServiceServer reset_srv;
	ros::ServiceServer checksum_srv;

	//! This type used in servicies to store 'data' fileds.
	typedef std::vector<uint8_t> V_FileData;

	enum OpState {
		OP_IDLE,
		OP_ACK,
		OP_LIST,
		OP_OPEN,
		OP_READ,
		OP_WRITE,
		OP_CHECKSUM
	};

	OpState op_state;
	uint16_t last_send_seqnr;	//!< seqNumber for send.
	uint32_t active_session;	//!< session id of current operation

	std::mutex cond_mutex;
	std::condition_variable cond;	//!< wait condvar
	bool is_error;			//!< error signaling flag (timeout/proto error)
	int r_errno;			//!< store errno from server

	// FTP:List
	uint32_t list_offset;
	std::string list_path;
	std::vector<mavros_msgs::FileEntry> list_entries;

	// FTP:Open / FTP:Close
	std::string open_path;
	size_t open_size;
	std::map<std::string, uint32_t> session_file_map;

	// FTP:Read
	size_t read_size;
	uint32_t read_offset;
	V_FileData read_buffer;

	// FTP:Write
	uint32_t write_offset;
	V_FileData write_buffer;
	V_FileData::iterator write_it;

	// FTP:CalcCRC32
	uint32_t checksum_crc32;

	// Timeouts,
	// computed as x4 time that needed for transmission of
	// one message at 57600 baud rate
	static constexpr int LIST_TIMEOUT_MS = 5000;
	static constexpr int OPEN_TIMEOUT_MS = 200;
	static constexpr int CHUNK_TIMEOUT_MS = 200;

	//! Maximum difference between allocated space and used
	static constexpr size_t MAX_RESERVE_DIFF = 0x10000;

	//! @todo exchange speed calculation
	//! @todo diagnostics

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
			go_idle(true, EILSEQ);
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
			go_idle(true, EBADRQC);
		}
	}

	void handle_req_ack(FTPRequest &req) {
		switch (op_state) {
		case OP_IDLE:		send_reset();			break;
		case OP_ACK:		go_idle(false);			break;
		case OP_LIST:		handle_ack_list(req);		break;
		case OP_OPEN:		handle_ack_open(req);		break;
		case OP_READ:		handle_ack_read(req);		break;
		case OP_WRITE:		handle_ack_write(req);		break;
		case OP_CHECKSUM:	handle_ack_checksum(req);	break;
		default:
			ROS_ERROR_NAMED("ftp", "FTP: wrong op_state");
			go_idle(true, EBADRQC);
		}
	}

	void handle_req_nack(FTPRequest &req) {
		auto hdr = req.header();
		auto error_code = static_cast<FTPRequest::ErrorCode>(req.data()[0]);
		OpState prev_op = op_state;

		ROS_ASSERT(hdr->size == 1 || (error_code == FTPRequest::kErrFailErrno && hdr->size == 2));

		op_state = OP_IDLE;
		if (error_code == FTPRequest::kErrFailErrno)
			r_errno = req.data()[1];
		// translate other protocol errors to errno
		else if (error_code == FTPRequest::kErrFail)
			r_errno = EFAULT;
		else if (error_code == FTPRequest::kErrInvalidDataSize)
			r_errno = EMSGSIZE;
		else if (error_code == FTPRequest::kErrInvalidSession)
			r_errno = EBADFD;
		else if (error_code == FTPRequest::kErrNoSessionsAvailable)
			r_errno = EMFILE;
		else if (error_code == FTPRequest::kErrUnknownCommand)
			r_errno = ENOSYS;

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

		ROS_ERROR_NAMED("ftp", "FTP: NAK: %u Opcode: %u State: %u Errno: %d (%s)",
				error_code, hdr->req_opcode, prev_op, r_errno, strerror(r_errno));
		go_idle(true);
	}

	void handle_ack_list(FTPRequest &req) {
		auto hdr = req.header();

		ROS_DEBUG_NAMED("ftp", "FTP:m: ACK List SZ(%u) OFF(%u)", hdr->size, hdr->offset);
		if (hdr->offset != list_offset) {
			ROS_ERROR_NAMED("ftp", "FTP: Wring list offset, req %u, ret %u",
					list_offset, hdr->offset);
			go_idle(true, EBADE);
			return;
		}

		uint8_t off = 0;
		uint32_t n_list_entries = 0;

		while (off < hdr->size) {
			const char *ptr = req.data_c() + off;
			const size_t bytes_left = hdr->size - off;

			size_t slen = strnlen(ptr, bytes_left);
			if ((ptr[0] == FTPRequest::DIRENT_SKIP && slen > 1) ||
					(ptr[0] != FTPRequest::DIRENT_SKIP && slen < 2)) {
				ROS_ERROR_NAMED("ftp", "FTP: Incorrect list entry: %s", ptr);
				go_idle(true, ERANGE);
				return;
			}
			else if (slen == bytes_left) {
				ROS_ERROR_NAMED("ftp", "FTP: Missing NULL termination in list entry");
				go_idle(true, EOVERFLOW);
				return;
			}

			if (ptr[0] == FTPRequest::DIRENT_FILE ||
					ptr[0] == FTPRequest::DIRENT_DIR) {
				add_dirent(ptr, slen);
			}
			else if (ptr[0] == FTPRequest::DIRENT_SKIP) {
				// do nothing
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

		ROS_DEBUG_NAMED("ftp", "FTP:m: ACK Open OPCODE(%u)", hdr->req_opcode);
		ROS_ASSERT(hdr->size == sizeof(uint32_t));
		open_size = *req.data_u32();

		ROS_INFO_NAMED("ftp", "FTP:Open %s: success, session %u, size %zu",
				open_path.c_str(), hdr->session, open_size);
		session_file_map.insert(std::make_pair(open_path, hdr->session));
		go_idle(false);
	}

	void handle_ack_read(FTPRequest &req) {
		auto hdr = req.header();

		ROS_DEBUG_NAMED("ftp", "FTP:m: ACK Read SZ(%u)", hdr->size);
		if (hdr->session != active_session) {
			ROS_ERROR_NAMED("ftp", "FTP:Read unexpected session");
			go_idle(true, EBADSLT);
			return;
		}

		if (hdr->offset != read_offset) {
			ROS_ERROR_NAMED("ftp", "FTP:Read different offset");
			go_idle(true, EBADE);
			return;
		}

		// kCmdReadFile return cunks of DATA_MAXSZ or smaller (last chunk)
		// We requested specific amount of data, that can be smaller,
		// but not larger.
		const size_t bytes_left = read_size - read_buffer.size();
		const size_t bytes_to_copy = std::min<size_t>(bytes_left, hdr->size);

		read_buffer.insert(read_buffer.end(), req.data(), req.data() + bytes_to_copy);

		if (bytes_to_copy == FTPRequest::DATA_MAXSZ) {
			// Possibly more data
			read_offset += bytes_to_copy;
			send_read_command();
		}
		else
			read_file_end();
	}

	void handle_ack_write(FTPRequest &req) {
		auto hdr = req.header();

		ROS_DEBUG_NAMED("ftp", "FTP:m: ACK Write SZ(%u)", hdr->size);
		if (hdr->session != active_session) {
			ROS_ERROR_NAMED("ftp", "FTP:Write unexpected session");
			go_idle(true, EBADSLT);
			return;
		}

		if (hdr->offset != write_offset) {
			ROS_ERROR_NAMED("ftp", "FTP:Write different offset");
			go_idle(true, EBADE);
			return;
		}

		ROS_ASSERT(hdr->size == sizeof(uint32_t));
		const size_t bytes_written = *req.data_u32();

		// check that reported size not out of range
		const size_t bytes_left_before_advance = std::distance(write_it, write_buffer.end());
		ROS_ASSERT_MSG(bytes_written <= bytes_left_before_advance, "Bad write size");
		ROS_ASSERT(bytes_written != 0);

		// move iterator to written size
		std::advance(write_it, bytes_written);

		const size_t bytes_to_copy = write_bytes_to_copy();
		if (bytes_to_copy > 0) {
			// More data to write
			write_offset += bytes_written;
			send_write_command(bytes_to_copy);
		}
		else
			write_file_end();
	}

	void handle_ack_checksum(FTPRequest &req) {
		auto hdr = req.header();

		ROS_DEBUG_NAMED("ftp", "FTP:m: ACK CalcFileCRC32 OPCODE(%u)", hdr->req_opcode);
		ROS_ASSERT(hdr->size == sizeof(uint32_t));
		checksum_crc32 = *req.data_u32();

		ROS_DEBUG_NAMED("ftp", "FTP:Checksum: success, crc32: 0x%08x", checksum_crc32);
		go_idle(false);
	}

	/* -*- send helpers -*- */

	/**
	 * @brief Go to IDLE mode
	 *
	 * @param is_error_ mark that caused in error case
	 * @param r_errno_ set r_errno in error case
	 */
	void go_idle(bool is_error_, int r_errno_ = 0) {
		op_state = OP_IDLE;
		is_error = is_error_;
		if (is_error && r_errno_ != 0) r_errno = r_errno_;
		else if (!is_error) r_errno = 0;
		cond.notify_all();
	}

	void send_reset() {
		ROS_DEBUG_NAMED("ftp", "FTP:m: kCmdResetSessions");
		if (!session_file_map.empty()) {
			ROS_WARN_NAMED("ftp", "FTP: Reset closes %zu sessons",
					session_file_map.size());
			session_file_map.clear();
		}

		op_state = OP_ACK;
		FTPRequest req(FTPRequest::kCmdResetSessions);
		req.send(uas, last_send_seqnr);
	}

	/// Send any command with string payload (usually file/dir path)
	inline void send_any_path_command(FTPRequest::Opcode op, const std::string &debug_msg, std::string &path, uint32_t offset) {
		ROS_DEBUG_STREAM_NAMED("ftp", "FTP:m: " << debug_msg << path << " off: " << offset);
		FTPRequest req(op);
		req.header()->offset = offset;
		req.set_data_string(path);
		req.send(uas, last_send_seqnr);
	}

	void send_list_command() {
		send_any_path_command(FTPRequest::kCmdListDirectory, "kCmdListDirectory: ", list_path, list_offset);
	}

	void send_open_ro_command() {
		send_any_path_command(FTPRequest::kCmdOpenFileRO, "kCmdOpenFileRO: ", open_path, 0);
	}

	void send_open_wo_command() {
		send_any_path_command(FTPRequest::kCmdOpenFileWO, "kCmdOpenFileWO: ", open_path, 0);
	}

	void send_create_command() {
		send_any_path_command(FTPRequest::kCmdCreateFile, "kCmdCreateFile: ", open_path, 0);
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

	void send_write_command(const size_t bytes_to_copy) {
		// write chunk from write_buffer [write_it..bytes_to_copy]
		ROS_DEBUG_STREAM_NAMED("ftp", "FTP:m: kCmdWriteFile: " << active_session << " off: " << write_offset << " sz: " << bytes_to_copy);
		FTPRequest req(FTPRequest::kCmdWriteFile, active_session);
		req.header()->offset = write_offset;
		req.header()->size = bytes_to_copy;
		std::copy(write_it, write_it + bytes_to_copy, req.data());
		req.send(uas, last_send_seqnr);
	}

	void send_remove_command(std::string &path) {
		send_any_path_command(FTPRequest::kCmdRemoveFile, "kCmdRemoveFile: ", path, 0);
	}

	bool send_rename_command(std::string &old_path, std::string &new_path) {
		std::ostringstream os;
		os << old_path;
		os << '\0';
		os << new_path;

		std::string paths = os.str();
		if (paths.size() >= FTPRequest::DATA_MAXSZ) {
			ROS_ERROR_NAMED("ftp", "FTP: rename file paths is too long: %zu", paths.size());
			r_errno = ENAMETOOLONG;
			return false;
		}

		send_any_path_command(FTPRequest::kCmdRename, "kCmdRename: ", paths, 0);
		return true;
	}

	void send_truncate_command(std::string &path, size_t length) {
		send_any_path_command(FTPRequest::kCmdTruncateFile, "kCmdTruncateFile: ", path, length);
	}

	void send_create_dir_command(std::string &path) {
		send_any_path_command(FTPRequest::kCmdCreateDirectory, "kCmdCreateDirectory: ", path, 0);
	}

	void send_remove_dir_command(std::string &path) {
		send_any_path_command(FTPRequest::kCmdRemoveDirectory, "kCmdRemoveDirectory: ", path, 0);
	}

	void send_calc_file_crc32_command(std::string &path) {
		send_any_path_command(FTPRequest::kCmdCalcFileCRC32, "kCmdCalcFileCRC32: ", path, 0);
	}

	/* -*- helpers -*- */

	void add_dirent(const char *ptr, size_t slen) {
		mavros_msgs::FileEntry ent;
		ent.size = 0;

		if (ptr[0] == FTPRequest::DIRENT_DIR) {
			ent.name.assign(ptr + 1, slen - 1);
			ent.type = mavros_msgs::FileEntry::TYPE_DIRECTORY;

			ROS_DEBUG_STREAM_NAMED("ftp", "FTP:List Dir: " << ent.name);
		}
		else {
			// ptr[0] == FTPRequest::DIRENT_FILE
			std::string name_size(ptr + 1, slen - 1);

			auto sep_it = std::find(name_size.begin(), name_size.end(), '\t');
			ent.name.assign(name_size.begin(), sep_it);
			ent.type = mavros_msgs::FileEntry::TYPE_FILE;

			if (sep_it != name_size.end()) {
				name_size.erase(name_size.begin(), sep_it + 1);
				if (name_size.size() != 0)
					ent.size = std::stoi(name_size);
			}

			ROS_DEBUG_STREAM_NAMED("ftp", "FTP:List File: " << ent.name << " SZ: " << ent.size);
		}

		list_entries.push_back(ent);
	}

	void list_directory_end() {
		ROS_DEBUG_NAMED("ftp", "FTP:List done");
		go_idle(false);
	}

	void list_directory(std::string &path) {
		list_offset = 0;
		list_path = path;
		list_entries.clear();
		op_state = OP_LIST;

		send_list_command();
	}

	bool open_file(std::string &path, int mode) {
		open_path = path;
		open_size = 0;
		op_state = OP_OPEN;

		if (mode == mavros_msgs::FileOpenRequest::MODE_READ)
			send_open_ro_command();
		else if (mode == mavros_msgs::FileOpenRequest::MODE_WRITE)
			send_open_wo_command();
		else if (mode == mavros_msgs::FileOpenRequest::MODE_CREATE)
			send_create_command();
		else {
			ROS_ERROR_NAMED("ftp", "FTP: Unsupported open mode: %d", mode);
			op_state = OP_IDLE;
			r_errno = EINVAL;
			return false;
		}

		return true;
	}

	bool close_file(std::string &path) {
		auto it = session_file_map.find(path);
		if (it == session_file_map.end()) {
			ROS_ERROR_NAMED("ftp", "FTP:Close %s: not opened", path.c_str());
			r_errno = EBADF;
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

	bool read_file(std::string &path, size_t off, size_t len) {
		auto it = session_file_map.find(path);
		if (it == session_file_map.end()) {
			ROS_ERROR_NAMED("ftp", "FTP:Read %s: not opened", path.c_str());
			r_errno = EBADF;
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

	void write_file_end() {
		ROS_DEBUG_NAMED("ftp", "FTP:Write done");
		go_idle(false);
	}

	bool write_file(std::string &path, size_t off, V_FileData &data) {
		auto it = session_file_map.find(path);
		if (it == session_file_map.end()) {
			ROS_ERROR_NAMED("ftp", "FTP:Write %s: not opened", path.c_str());
			r_errno = EBADF;
			return false;
		}

		op_state = OP_WRITE;
		active_session = it->second;
		write_offset = off;
		write_buffer = std::move(data);
		write_it = write_buffer.begin();

		send_write_command(write_bytes_to_copy());
		return true;
	}

	void remove_file(std::string &path) {
		op_state = OP_ACK;
		send_remove_command(path);
	}

	bool rename_(std::string &old_path, std::string &new_path) {
		op_state = OP_ACK;
		return send_rename_command(old_path, new_path);
	}

	void truncate_file(std::string &path, size_t length) {
		op_state = OP_ACK;
		send_truncate_command(path, length);
	}

	void create_directory(std::string &path) {
		op_state = OP_ACK;
		send_create_dir_command(path);
	}

	void remove_directory(std::string &path) {
		op_state = OP_ACK;
		send_remove_dir_command(path);
	}

	void checksum_crc32_file(std::string &path) {
		op_state = OP_CHECKSUM;
		checksum_crc32 = 0;
		send_calc_file_crc32_command(path);
	}

	static constexpr int compute_rw_timeout(size_t len) {
		return CHUNK_TIMEOUT_MS * (len / FTPRequest::DATA_MAXSZ + 1);
	}

	size_t write_bytes_to_copy() {
		return std::min<size_t>(std::distance(write_it, write_buffer.end()),
				FTPRequest::DATA_MAXSZ);
	}

	bool wait_completion(const int msecs) {
		std::unique_lock<std::mutex> lock(cond_mutex);

		bool is_timedout = cond.wait_for(lock, std::chrono::milliseconds(msecs))
				== std::cv_status::timeout;

		if (is_timedout) {
			// If timeout occurs don't forget to reset state
			op_state = OP_IDLE;
			r_errno = ETIMEDOUT;
			return false;
		}
		else
			// if go_idle() occurs before timeout
			return !is_error;
	}

	/* -*- service callbacks -*- */

	/**
	 * Service handler common header code.
	 */
#define SERVICE_IDLE_CHECK()				\
	if (op_state != OP_IDLE) {			\
		ROS_ERROR_NAMED("ftp", "FTP: Busy");	\
		return false;				\
	}

	bool list_cb(mavros_msgs::FileList::Request &req,
			mavros_msgs::FileList::Response &res) {
		SERVICE_IDLE_CHECK();

		list_directory(req.dir_path);
		res.success = wait_completion(LIST_TIMEOUT_MS);
		res.r_errno = r_errno;
		if (res.success) {
			res.list = std::move(list_entries);
			list_entries.clear();	// not shure that it's needed
		}

		return true;
	}

	bool open_cb(mavros_msgs::FileOpen::Request &req,
			mavros_msgs::FileOpen::Response &res) {
		SERVICE_IDLE_CHECK();

		// only one session per file
		auto it = session_file_map.find(req.file_path);
		if (it != session_file_map.end()) {
			ROS_ERROR_NAMED("ftp", "FTP: File %s: already opened",
					req.file_path.c_str());
			return false;
		}

		res.success = open_file(req.file_path, req.mode);
		if (res.success) {
			res.success = wait_completion(OPEN_TIMEOUT_MS);
			res.size = open_size;
		}
		res.r_errno = r_errno;

		return true;
	}

	bool close_cb(mavros_msgs::FileClose::Request &req,
			mavros_msgs::FileClose::Response &res) {
		SERVICE_IDLE_CHECK();

		res.success = close_file(req.file_path);
		if (res.success) {
			res.success = wait_completion(OPEN_TIMEOUT_MS);
		}
		res.r_errno = r_errno;

		return true;
	}

	bool read_cb(mavros_msgs::FileRead::Request &req,
			mavros_msgs::FileRead::Response &res) {
		SERVICE_IDLE_CHECK();

		res.success = read_file(req.file_path, req.offset, req.size);
		if (res.success)
			res.success = wait_completion(compute_rw_timeout(req.size));
		if (res.success) {
			res.data = std::move(read_buffer);
			read_buffer.clear();	// same as for list_entries
		}
		res.r_errno = r_errno;

		return true;
	}

	bool write_cb(mavros_msgs::FileWrite::Request &req,
			mavros_msgs::FileWrite::Response &res) {
		SERVICE_IDLE_CHECK();

		const size_t data_size = req.data.size();
		res.success = write_file(req.file_path, req.offset, req.data);
		if (res.success) {
			res.success = wait_completion(compute_rw_timeout(data_size));
		}
		write_buffer.clear();
		res.r_errno = r_errno;

		return true;
	}

	bool remove_cb(mavros_msgs::FileRemove::Request &req,
			mavros_msgs::FileRemove::Response &res) {
		SERVICE_IDLE_CHECK();

		remove_file(req.file_path);
		res.success = wait_completion(OPEN_TIMEOUT_MS);
		res.r_errno = r_errno;

		return true;
	}

	bool rename_cb(mavros_msgs::FileRename::Request &req,
			mavros_msgs::FileRename::Response &res) {
		SERVICE_IDLE_CHECK();

		res.success = rename_(req.old_path, req.new_path);
		if (res.success) {
			res.success = wait_completion(OPEN_TIMEOUT_MS);
		}
		res.r_errno = r_errno;

		return true;
	}


	bool truncate_cb(mavros_msgs::FileTruncate::Request &req,
			mavros_msgs::FileTruncate::Response &res) {
		SERVICE_IDLE_CHECK();

		// Note: emulated truncate() can take a while
		truncate_file(req.file_path, req.length);
		res.success = wait_completion(LIST_TIMEOUT_MS * 5);
		res.r_errno = r_errno;

		return true;
	}

	bool mkdir_cb(mavros_msgs::FileMakeDir::Request &req,
			mavros_msgs::FileMakeDir::Response &res) {
		SERVICE_IDLE_CHECK();

		create_directory(req.dir_path);
		res.success = wait_completion(OPEN_TIMEOUT_MS);
		res.r_errno = r_errno;

		return true;
	}

	bool rmdir_cb(mavros_msgs::FileRemoveDir::Request &req,
			mavros_msgs::FileRemoveDir::Response &res) {
		SERVICE_IDLE_CHECK();

		remove_directory(req.dir_path);
		res.success = wait_completion(OPEN_TIMEOUT_MS);
		res.r_errno = r_errno;

		return true;
	}

	bool checksum_cb(mavros_msgs::FileChecksum::Request &req,
			mavros_msgs::FileChecksum::Response &res) {
		SERVICE_IDLE_CHECK();

		checksum_crc32_file(req.file_path);
		res.success = wait_completion(LIST_TIMEOUT_MS);
		res.crc32 = checksum_crc32;
		res.r_errno = r_errno;

		return true;
	}

#undef SERVICE_IDLE_CHECK

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
};	// namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::FTPPlugin, mavplugin::MavRosPlugin)
