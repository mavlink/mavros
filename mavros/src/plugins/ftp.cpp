/*
 * Copyright 2014,2016,2021 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */
/**
 * @brief FTP plugin
 * @file ftp.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */

#include <chrono>
#include <cerrno>
#include <cstddef>
#include <cstring>
#include <condition_variable>
#include <string>
#include <vector>
#include <map>
#include <utility>
#include <algorithm>
#include <type_traits>

#include "rcpputils/asserts.hpp"
#include "mavros/mavros_uas.hpp"
#include "mavros/plugin.hpp"
#include "mavros/plugin_filter.hpp"

#include "std_srvs/srv/empty.hpp"
#include "mavros_msgs/msg/file_entry.hpp"
#include "mavros_msgs/srv/file_list.hpp"
#include "mavros_msgs/srv/file_open.hpp"
#include "mavros_msgs/srv/file_close.hpp"
#include "mavros_msgs/srv/file_read.hpp"
#include "mavros_msgs/srv/file_write.hpp"
#include "mavros_msgs/srv/file_remove.hpp"
#include "mavros_msgs/srv/file_make_dir.hpp"
#include "mavros_msgs/srv/file_remove_dir.hpp"
#include "mavros_msgs/srv/file_truncate.hpp"
#include "mavros_msgs/srv/file_rename.hpp"
#include "mavros_msgs/srv/file_checksum.hpp"

// enable debugging messages
// #define FTP_LL_DEBUG

namespace mavros
{
namespace std_plugins
{
using namespace std::placeholders;      // NOLINT
using utils::enum_value;

/**
 * @brief FTP Request message abstraction class
 *
 * @note This class is not portable, and works on little-endian machines only.
 */
class FTPRequest : public mavlink::common::msg::FILE_TRANSFER_PROTOCOL
{
public:
  /// @brief This is the payload which is in mavlink_file_transfer_protocol_t.payload.
  /// We pad the structure ourselves to 32 bit alignment to avoid usage of any pack pragmas.
  struct PayloadHeader
  {
    uint16_t seqNumber;         ///< sequence number for message
    uint8_t session;            ///< Session id for read and write commands
    uint8_t opcode;             ///< Command opcode
    uint8_t size;               ///< Size of data
    uint8_t req_opcode;         ///< Request opcode returned in kRspAck, kRspNak message
    uint8_t padding[2];         ///< 32 bit alignment padding
    uint32_t offset;            ///< Offsets for List and Read commands
  };

  static_assert(sizeof(PayloadHeader) == 12, "PayloadHeader must match the 12-byte wire layout");
  static_assert(std::is_trivially_copyable_v<PayloadHeader>);
  static_assert(offsetof(PayloadHeader, seqNumber) == 0, "seqNumber offset mismatch");
  static_assert(offsetof(PayloadHeader, session) == 2, "session offset mismatch");
  static_assert(offsetof(PayloadHeader, opcode) == 3, "opcode offset mismatch");
  static_assert(offsetof(PayloadHeader, size) == 4, "size offset mismatch");
  static_assert(offsetof(PayloadHeader, req_opcode) == 5, "req_opcode offset mismatch");
  static_assert(offsetof(PayloadHeader, padding) == 6, "padding offset mismatch");
  static_assert(offsetof(PayloadHeader, offset) == 8, "offset offset mismatch");

  /// @brief Command opcodes
  enum Opcode : uint8_t
  {
    kCmdNone,                   ///< ignored, always acked
    kCmdTerminateSession,       ///< Terminates open Read session
    kCmdResetSessions,          ///< Terminates all open Read sessions
    kCmdListDirectory,          ///< List files in <path> from <offset>
    kCmdOpenFileRO,             ///< Opens file at <path> for reading, returns <session>
    kCmdReadFile,               ///< Reads <size> bytes from <offset> in <session>
    kCmdCreateFile,             ///< Creates file at <path> for writing, returns <session>
    kCmdWriteFile,              ///< Writes <size> bytes to <offset> in <session>
    kCmdRemoveFile,             ///< Remove file at <path>
    kCmdCreateDirectory,        ///< Creates directory at <path>
    kCmdRemoveDirectory,        ///< Removes Directory at <path>, must be empty
    kCmdOpenFileWO,             ///< Opens file at <path> for writing, returns <session>
    kCmdTruncateFile,           ///< Truncate file at <path> to <offset> length
    kCmdRename,                 ///< Rename <path1> to <path2>
    kCmdCalcFileCRC32,          ///< Calculate CRC32 for file at <path>
    kCmdBurstReadFile,          ///< Burst download session file

    kRspAck = 128,              ///< Ack response
    kRspNak                     ///< Nak response
  };

  /// @brief Error codes returned in Nak response.
  enum ErrorCode : uint8_t
  {
    kErrNone,
    kErrFail,                   ///< Unknown failure
    kErrFailErrno,              ///< Command failed, errno sent back in PayloadHeader.data[1]
    kErrInvalidDataSize,        ///< PayloadHeader.size is invalid
    kErrInvalidSession,         ///< Session is not currently open
    kErrNoSessionsAvailable,    ///< All available Sessions in use
    kErrEOF,                    ///< Offset past end of file for List and Read commands
    kErrUnknownCommand,         ///< Unknown command opcode
    kErrFailFileExists,         ///< File exists already
    kErrFailFileProtected,      ///< File is write protected
    kErrFileNotFound            ///< File not found
  };

  static const char DIRENT_FILE = 'F';
  static const char DIRENT_DIR = 'D';
  static const char DIRENT_SKIP = 'S';
  //! payload.size() - header bytes
  static const uint8_t DATA_MAXSZ = 251 - sizeof(PayloadHeader);

  inline const uint8_t * raw_payload() const
  {
    return payload.data();
  }

  inline uint8_t * raw_payload()
  {
    return payload.data();
  }

  /**
   * @brief Copy of the payload header
   *
   * The payload buffer has no alignment guarantee, so the header is copied
   * out instead of being accessed in place.
   */
  inline PayloadHeader header() const
  {
    PayloadHeader hdr;
    memcpy(&hdr, payload.data(), sizeof(hdr));
    return hdr;
  }

  /// @brief Write the header back into the payload
  inline void set_header(const PayloadHeader & hdr)
  {
    memcpy(payload.data(), &hdr, sizeof(hdr));
  }

  inline const uint8_t * data() const
  {
    return payload.data() + sizeof(PayloadHeader);
  }

  inline uint8_t * data()
  {
    return payload.data() + sizeof(PayloadHeader);
  }

  inline const char * data_c() const
  {
    return reinterpret_cast<const char *>(data());
  }

  inline char * data_c()
  {
    return reinterpret_cast<char *>(data());
  }

  /// @brief First 32-bit value of the data, copied out since the payload has no alignment guarantee
  inline uint32_t data_u32() const
  {
    uint32_t value;
    memcpy(&value, data(), sizeof(value));
    return value;
  }

  /**
   * @brief Copy string to payload
   *
   * @param[in] s  payload string
   * @note this function allows null termination inside string
   *       it is used to send multiple strings in one message
   */
  void set_data_string(const std::string & s)
  {
    size_t sz = (s.size() < DATA_MAXSZ - 1) ? s.size() : DATA_MAXSZ - 1;

    memcpy(data_c(), s.c_str(), sz);
    data_c()[sz] = '\0';
    auto hdr = header();
    hdr.size = sz;
    set_header(hdr);
  }

  uint8_t get_target_system_id() const
  {
    return target_system;
  }

  /**
   * @brief Decode and check target system
   */
  bool decode_valid([[maybe_unused]] plugin::UASPtr uas)
  {
#ifdef FTP_LL_DEBUG
    const auto hdr = header();
    RCLCPP_DEBUG(
      rclcpp::get_logger("ftp.request"),
      "FTP:rm: SEQ(%u) SESS(%u) OPCODE(%u) RQOP(%u) SZ(%u) OFF(%u)",
      hdr.seqNumber, hdr.session, hdr.opcode, hdr.req_opcode, hdr.size, hdr.offset);
#endif

    // return uas->get_system_id() == target_system;
    return true;  // XXX TODO(vooon): probably whole method is not needed anymore
  }

  /**
   * @brief Encode and send message
   */
  void send(plugin::UASPtr uas, uint16_t seqNumber)
  {
    target_network = 0;
    target_system = uas->get_tgt_system();
    target_component = uas->get_tgt_component();

    auto hdr = header();
    hdr.seqNumber = seqNumber;
    hdr.req_opcode = kCmdNone;
    set_header(hdr);

#ifdef FTP_LL_DEBUG
    RCLCPP_DEBUG(
      rclcpp::get_logger("ftp.request"), "FTP:sm: SEQ(%u) SESS(%u) OPCODE(%u) SZ(%u) OFF(%u)",
      hdr.seqNumber, hdr.session, hdr.opcode, hdr.size, hdr.offset);
#endif

    uas->send_message(*this);
  }

  FTPRequest()
  : mavlink::common::msg::FILE_TRANSFER_PROTOCOL{}
  {}

  explicit FTPRequest(Opcode op, uint8_t session = 0)
  : mavlink::common::msg::FILE_TRANSFER_PROTOCOL{}
  {
    PayloadHeader hdr{};
    hdr.session = session;
    hdr.opcode = op;
    set_header(hdr);
  }
};


/**
 * @brief FTP plugin.
 * @plugin ftp
 *
 * Implements the
 * [MAVLink File Transfer Protocol](https://mavlink.io/en/services/ftp.html).
 */
class FTPPlugin : public plugin::Plugin
{
public:
  explicit FTPPlugin(plugin::UASPtr uas_)
  : Plugin(uas_, "ftp"),
    op_state(OP::IDLE),
    last_send_seqnr(0),
    active_session(0),
    is_error(false),
    r_errno(0),
    rx_progress(0),
    list_offset(0),
    open_size(0),
    read_size(0),
    read_offset(0),
    read_buffer{},
    read_burst(false),
    burst_supported(true),
    burst_resyncs(0),
    write_offset(0),
    write_buffer{},
    write_it{},
    write_chunk_size(0),
    checksum_crc32(0)
  {
    // since C++ generator do not produce field length defs make check explicit.
    FTPRequest r;
    rcpputils::assert_true((r.payload.size() - sizeof(FTPRequest::PayloadHeader)) == r.DATA_MAXSZ);

    //! List the contents of a directory on the FCU (FTP).
    list_srv =
      node->create_service<mavros_msgs::srv::FileList>(
      "~/list",
      std::bind(&FTPPlugin::list_cb, this, _1, _2));
    //! Open a file on the FCU for reading or writing (FTP).
    open_srv =
      node->create_service<mavros_msgs::srv::FileOpen>(
      "~/open",
      std::bind(&FTPPlugin::open_cb, this, _1, _2));
    //! Close an open file on the FCU (FTP).
    close_srv =
      node->create_service<mavros_msgs::srv::FileClose>(
      "~/close",
      std::bind(&FTPPlugin::close_cb, this, _1, _2));
    //! Read data from an open file on the FCU (FTP).
    read_srv =
      node->create_service<mavros_msgs::srv::FileRead>(
      "~/read",
      std::bind(&FTPPlugin::read_cb, this, _1, _2));
    //! Write data to an open file on the FCU (FTP).
    write_srv =
      node->create_service<mavros_msgs::srv::FileWrite>(
      "~/write",
      std::bind(&FTPPlugin::write_cb, this, _1, _2));
    //! Create a directory on the FCU (FTP).
    mkdir_srv =
      node->create_service<mavros_msgs::srv::FileMakeDir>(
      "~/mkdir",
      std::bind(&FTPPlugin::mkdir_cb, this, _1, _2));
    //! Remove a directory on the FCU (FTP).
    rmdir_srv =
      node->create_service<mavros_msgs::srv::FileRemoveDir>(
      "~/rmdir",
      std::bind(&FTPPlugin::rmdir_cb, this, _1, _2));
    //! Remove a file on the FCU (FTP).
    remove_srv =
      node->create_service<mavros_msgs::srv::FileRemove>(
      "~/remove",
      std::bind(&FTPPlugin::remove_cb, this, _1, _2));
    //! Truncate a file to a given length on the FCU (FTP).
    truncate_srv =
      node->create_service<mavros_msgs::srv::FileTruncate>(
      "~/truncate",
      std::bind(&FTPPlugin::truncate_cb, this, _1, _2));
    //! Reset the FTP session on both sides (FTP).
    reset_srv =
      node->create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&FTPPlugin::reset_cb, this, _1, _2));
    //! Rename a file on the FCU (FTP).
    rename_srv =
      node->create_service<mavros_msgs::srv::FileRename>(
      "~/rename",
      std::bind(&FTPPlugin::rename_cb, this, _1, _2));
    //! Calculate the CRC32 checksum of a file on the FCU (FTP).
    checksum_srv =
      node->create_service<mavros_msgs::srv::FileChecksum>(
      "~/checksum",
      std::bind(&FTPPlugin::checksum_cb, this, _1, _2));
  }

  Subscriptions get_subscriptions() override
  {
    return {
      make_handler(&FTPPlugin::handle_file_transfer_protocol),
    };
  }

private:
  rclcpp::Service<mavros_msgs::srv::FileList>::SharedPtr list_srv;
  rclcpp::Service<mavros_msgs::srv::FileOpen>::SharedPtr open_srv;
  rclcpp::Service<mavros_msgs::srv::FileClose>::SharedPtr close_srv;
  rclcpp::Service<mavros_msgs::srv::FileRead>::SharedPtr read_srv;
  rclcpp::Service<mavros_msgs::srv::FileWrite>::SharedPtr write_srv;
  rclcpp::Service<mavros_msgs::srv::FileMakeDir>::SharedPtr mkdir_srv;
  rclcpp::Service<mavros_msgs::srv::FileRemoveDir>::SharedPtr rmdir_srv;
  rclcpp::Service<mavros_msgs::srv::FileRemove>::SharedPtr remove_srv;
  rclcpp::Service<mavros_msgs::srv::FileRename>::SharedPtr rename_srv;
  rclcpp::Service<mavros_msgs::srv::FileTruncate>::SharedPtr truncate_srv;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv;
  rclcpp::Service<mavros_msgs::srv::FileChecksum>::SharedPtr checksum_srv;

  //! This type used in services to store 'data' fields.
  typedef std::vector<uint8_t> V_FileData;

  enum class OP
  {
    IDLE,
    ACK,
    LIST,
    OPEN,
    READ,
    WRITE,
    CHECKSUM
  };

  OP op_state;
  uint16_t last_send_seqnr;             //!< seqNumber for send.
  uint32_t active_session;              //!< session id of current operation

  std::mutex cond_mutex;
  std::condition_variable cond;         //!< wait condvar
  bool is_error;                        //!< error signaling flag (timeout/proto error)
  int r_errno;                          //!< store errno from server
  uint32_t rx_progress;                 //!< accepted replies, timeouts count from the last one

  // FTP:List
  uint32_t list_offset;
  std::string list_path;
  std::vector<mavros_msgs::msg::FileEntry> list_entries;

  // FTP:Open / FTP:Close
  std::string open_path;
  size_t open_size;
  struct OpenFile
  {
    uint32_t session;
    size_t size;
  };
  std::map<std::string, OpenFile> session_file_map;

  // FTP:Read
  size_t read_size;
  uint32_t read_offset;
  V_FileData read_buffer;
  bool read_burst;
  bool burst_supported;
  int burst_resyncs;

  // FTP:Write
  uint32_t write_offset;
  V_FileData write_buffer;
  V_FileData::iterator write_it;
  size_t write_chunk_size;

  // FTP:CalcCRC32
  uint32_t checksum_crc32;

  // Timeouts, applied per stall (no reply), not per whole operation.
  // ArduPilot regularly needs more than 200 ms to service a request.
  static constexpr int LIST_TIMEOUT_MS = 5000;
  static constexpr int OPEN_TIMEOUT_MS = 1000;
  static constexpr int CHUNK_TIMEOUT_MS = 1000;

  //! Seqnr window of one burst, larger than any FCU burst (ArduPilot: up to 2000 packets)
  static constexpr uint16_t BURST_SEQ_WINDOW = 4096;
  //! Burst restarts after lost packets without any data in between
  static constexpr int MAX_BURST_RESYNCS = 10;

  //! Maximum difference between allocated space and used
  static constexpr size_t MAX_RESERVE_DIFF = 0x10000;

  //! @todo exchange speed calculation
  //! @todo diagnostics
  //! @todo multisession not present anymore

  /* -*- message handler -*- */

  //! handler for mavlink::common::msg::FILE_TRANSFER_PROTOCOL
  void handle_file_transfer_protocol(
    const mavlink::mavlink_message_t * msg [[maybe_unused]],
    FTPRequest & req,
    plugin::filter::SystemAndOk filter [[maybe_unused]])
  {
    if (!req.decode_valid(uas)) {
      // RCLCPP_DEBUG(get_logger(), "FTP: Wrong System Id, MY %u, TGT %u",
      //   uas->get_system_id(), req.get_target_system_id());
      return;
    }

    std::lock_guard<std::mutex> lock(cond_mutex);
    if (op_state == OP::IDLE) {
      return;
    }

    const auto hdr = req.header();
    if (hdr.size > FTPRequest::DATA_MAXSZ) {
      RCLCPP_ERROR(get_logger(), "FTP: payload size %u > %u", hdr.size, FTPRequest::DATA_MAXSZ);
      go_idle(true, EBADMSG);
      return;
    }

    // A late reply to a request that already timed out carries an older seqnr.
    // Burst replies count up from the expected seqnr, one per packet.
    const uint16_t incoming_seqnr = hdr.seqNumber;
    const uint16_t expected_seqnr = last_send_seqnr + 1;
    if (hdr.req_opcode == FTPRequest::kCmdBurstReadFile) {
      if (op_state != OP::READ || !read_burst) {
        return;
      }
      if (static_cast<uint16_t>(incoming_seqnr - expected_seqnr) >= BURST_SEQ_WINDOW) {
        // still streaming an earlier burst, which shows the FCU is alive
        rx_progress++;
        return;
      }
    } else if (incoming_seqnr != expected_seqnr) {
      RCLCPP_WARN(
        get_logger(), "FTP: Ignoring out of sync reply, seqnr: %u != %u",
        incoming_seqnr, expected_seqnr);
      return;
    }

    rx_progress++;

    // logic from QGCUASFileManager.cc
    if (hdr.opcode == FTPRequest::kRspAck) {
      handle_req_ack(req);
    } else if (hdr.opcode == FTPRequest::kRspNak) {
      handle_req_nack(req);
    } else {
      RCLCPP_ERROR(get_logger(), "FTP: Unknown request response: %u", hdr.opcode);
      go_idle(true, EBADRQC);
    }
  }

  void handle_req_ack(const FTPRequest & req)
  {
    switch (op_state) {
      case OP::ACK:           go_idle(false);                 break;
      case OP::LIST:          handle_ack_list(req);           break;
      case OP::OPEN:          handle_ack_open(req);           break;
      case OP::READ:          handle_ack_read(req);           break;
      case OP::WRITE:         handle_ack_write(req);          break;
      case OP::CHECKSUM:      handle_ack_checksum(req);       break;
      default:
        RCLCPP_ERROR(get_logger(), "FTP: wrong op_state");
        go_idle(true, EBADRQC);
    }
  }

  void handle_req_nack(const FTPRequest & req)
  {
    const auto hdr = req.header();
    auto error_code = static_cast<FTPRequest::ErrorCode>(req.data()[0]);
    auto prev_op = op_state;

    if (!(hdr.size == 1 || (error_code == FTPRequest::kErrFailErrno && hdr.size == 2))) {
      RCLCPP_ERROR(get_logger(), "FTP: NAK with bad payload size %u", hdr.size);
      go_idle(true, EBADMSG);
      return;
    }

    op_state = OP::IDLE;
    if (error_code == FTPRequest::kErrFailErrno) {
      r_errno = req.data()[1];

      /* translate other protocol errors to errno */
    } else if (error_code == FTPRequest::kErrFail) {
      r_errno = EFAULT;
    } else if (error_code == FTPRequest::kErrInvalidDataSize) {
      r_errno = EMSGSIZE;
    } else if (error_code == FTPRequest::kErrInvalidSession) {
      r_errno = EBADFD;
    } else if (error_code == FTPRequest::kErrNoSessionsAvailable) {
      r_errno = EMFILE;
    } else if (error_code == FTPRequest::kErrUnknownCommand) {
      r_errno = ENOSYS;
    } else if (error_code == FTPRequest::kErrFailFileExists) {
      r_errno = EEXIST;
    } else if (error_code == FTPRequest::kErrFailFileProtected) {
      r_errno = EACCES;
    } else if (error_code == FTPRequest::kErrFileNotFound) {
      r_errno = ENOENT;
    } else {
      r_errno = EIO;
    }

    // Some FCUs answer a listing past the last entry with an errno instead of EOF.
    // At offset 0 that is a real error (e.g. no such directory).
    const bool list_errno_end = list_offset > 0 &&
      (error_code == FTPRequest::kErrFileNotFound ||
      (error_code == FTPRequest::kErrFailErrno &&
      (r_errno == 0 || r_errno == ENOENT || r_errno == ERANGE)));

    if (prev_op == OP::READ && read_burst && error_code == FTPRequest::kErrUnknownCommand) {
      RCLCPP_WARN(get_logger(), "FTP: FCU does not support burst read, using kCmdReadFile");
      burst_supported = false;
      read_burst = false;
      op_state = OP::READ;
      send_read_command();
      return;
    }

    if (prev_op == OP::LIST && (error_code == FTPRequest::kErrEOF || list_errno_end)) {
      /* dir list done */
      list_directory_end();
      return;
    } else if (prev_op == OP::READ && error_code == FTPRequest::kErrEOF) {
      /* read done */
      read_file_end();
      return;
    }

    RCLCPP_ERROR(
      get_logger(), "FTP: NAK: %u Opcode: %u State: %u Errno: %d (%s)",
      error_code, hdr.req_opcode, enum_value(prev_op), r_errno, strerror(r_errno));
    go_idle(true);
  }

  void handle_ack_list(const FTPRequest & req)
  {
    const auto hdr = req.header();

    RCLCPP_DEBUG(get_logger(), "FTP:m: ACK List SZ(%u) OFF(%u)", hdr.size, hdr.offset);
    if (hdr.offset != list_offset) {
      RCLCPP_ERROR(
        get_logger(), "FTP: Wrong list offset, req %u, ret %u",
        list_offset, hdr.offset);
      go_idle(true, EBADE);
      return;
    }

    size_t off = 0;
    uint32_t n_list_entries = 0;

    while (off < hdr.size) {
      const char * ptr = req.data_c() + off;
      const size_t bytes_left = hdr.size - off;

      size_t slen = strnlen(ptr, bytes_left);
      if (slen == 0) {
        // ArduPilot terminates each entry with two NULs
        off++;
        continue;
      }

      if ((ptr[0] == FTPRequest::DIRENT_SKIP && slen > 1) ||
        (ptr[0] != FTPRequest::DIRENT_SKIP && slen < 2))
      {
        RCLCPP_ERROR(get_logger(), "FTP: Incorrect list entry: %s", ptr);
        go_idle(true, ERANGE);
        return;
      } else if (slen == bytes_left) {
        RCLCPP_ERROR(get_logger(), "FTP: Missing NULL termination in list entry");
        go_idle(true, EOVERFLOW);
        return;
      }

      if (ptr[0] == FTPRequest::DIRENT_FILE ||
        ptr[0] == FTPRequest::DIRENT_DIR)
      {
        if (!add_dirent(ptr, slen)) {
          RCLCPP_ERROR(get_logger(), "FTP: Incorrect list entry: %s", ptr);
          go_idle(true, ERANGE);
          return;
        }
      } else if (ptr[0] == FTPRequest::DIRENT_SKIP) {
        // do nothing
      } else {
        RCLCPP_WARN(get_logger(), "FTP: Unknown list entry: %s", ptr);
      }

      off += slen + 1;
      n_list_entries++;
    }

    if (hdr.size == 0) {
      // dir empty, we are done
      list_directory_end();
    } else if (n_list_entries == 0) {
      RCLCPP_ERROR(get_logger(), "FTP: List reply without entries");
      go_idle(true, ERANGE);
    } else {
      // Possibly more to come, try get more
      list_offset += n_list_entries;
      send_list_command();
    }
  }

  void handle_ack_open(const FTPRequest & req)
  {
    const auto hdr = req.header();

    RCLCPP_DEBUG(get_logger(), "FTP:m: ACK Open OPCODE(%u)", hdr.req_opcode);
    // ArduPilot ACKs OpenFileWO / CreateFile without a size
    if (hdr.size == sizeof(uint32_t)) {
      open_size = req.data_u32();
    } else if (hdr.size != 0) {
      RCLCPP_ERROR(get_logger(), "FTP:Open %s: bad ACK size %u", open_path.c_str(), hdr.size);
      go_idle(true, EBADMSG);
      return;
    }

    RCLCPP_INFO(
      get_logger(), "FTP:Open %s: success, session %u, size %zu",
      open_path.c_str(), hdr.session, open_size);
    session_file_map.insert(std::make_pair(open_path, OpenFile{hdr.session, open_size}));
    go_idle(false);
  }

  void handle_ack_read(const FTPRequest & req)
  {
    const auto hdr = req.header();
    auto lg = get_logger();

    RCLCPP_DEBUG(lg, "FTP:m: ACK Read SZ(%u)", hdr.size);
    if (hdr.session != active_session) {
      RCLCPP_ERROR(lg, "FTP:Read unexpected session");
      go_idle(true, EBADSLT);
      return;
    }

    if (read_burst && hdr.offset > read_offset) {
      // lost packet(s), restart the burst at the gap
      if (++burst_resyncs > MAX_BURST_RESYNCS) {
        RCLCPP_ERROR(lg, "FTP:Read too many lost burst packets");
        go_idle(true, EIO);
        return;
      }
      RCLCPP_DEBUG(lg, "FTP:Read burst gap at %u, got %u", read_offset, hdr.offset);
      send_burst_read_command();
      return;
    }

    if (hdr.offset != read_offset) {
      RCLCPP_ERROR(lg, "FTP:Read different offset");
      go_idle(true, EBADE);
      return;
    }

    // kCmdReadFile return cunks of DATA_MAXSZ or smaller (last chunk)
    // We requested specific amount of data, that can be smaller,
    // but not larger.
    const size_t bytes_left = read_size - read_buffer.size();
    const size_t bytes_to_copy = std::min<size_t>(bytes_left, hdr.size);

    read_buffer.insert(read_buffer.end(), req.data(), req.data() + bytes_to_copy);
    read_offset += bytes_to_copy;

    if (read_buffer.size() >= read_size) {
      read_file_end();
    } else if (read_burst) {
      // the FCU ends a burst at EOF (NAK) or after its own packet limit
      burst_resyncs = 0;
      if (hdr.padding[0]) {
        send_burst_read_command();
      }
    } else if (hdr.size == FTPRequest::DATA_MAXSZ) {
      // Possibly more data
      send_read_command();
    } else {
      read_file_end();
    }
  }

  void handle_ack_write(const FTPRequest & req)
  {
    const auto hdr = req.header();
    auto lg = get_logger();

    RCLCPP_DEBUG(lg, "FTP:m: ACK Write SZ(%u)", hdr.size);
    if (hdr.session != active_session) {
      RCLCPP_ERROR(lg, "FTP:Write unexpected session");
      go_idle(true, EBADSLT);
      return;
    }

    if (hdr.offset != write_offset) {
      RCLCPP_ERROR(lg, "FTP:Write different offset");
      go_idle(true, EBADE);
      return;
    }

    // ArduPilot ACKs WriteFile without a byte count: it writes the whole chunk or NAKs
    size_t bytes_written = write_chunk_size;
    if (hdr.size == sizeof(uint32_t)) {
      bytes_written = req.data_u32();
    } else if (hdr.size != 0) {
      RCLCPP_ERROR(lg, "FTP:Write bad ACK size %u", hdr.size);
      go_idle(true, EBADMSG);
      return;
    }

    // check that reported size not out of range
    const size_t bytes_left_before_advance = std::distance(write_it, write_buffer.end());
    if (bytes_written == 0 || bytes_written > bytes_left_before_advance) {
      RCLCPP_ERROR(lg, "FTP:Write bad write size %zu", bytes_written);
      go_idle(true, EBADMSG);
      return;
    }

    // move iterator to written size
    std::advance(write_it, bytes_written);

    const size_t bytes_to_copy = write_bytes_to_copy();
    if (bytes_to_copy > 0) {
      // More data to write
      write_offset += bytes_written;
      send_write_command(bytes_to_copy);
    } else {
      write_file_end();
    }
  }

  void handle_ack_checksum(const FTPRequest & req)
  {
    const auto hdr = req.header();
    auto lg = get_logger();

    RCLCPP_DEBUG(lg, "FTP:m: ACK CalcFileCRC32 OPCODE(%u)", hdr.req_opcode);
    if (hdr.size != sizeof(uint32_t)) {
      RCLCPP_ERROR(lg, "FTP:Checksum bad ACK size %u", hdr.size);
      go_idle(true, EBADMSG);
      return;
    }
    checksum_crc32 = req.data_u32();

    RCLCPP_DEBUG(lg, "FTP:Checksum: success, crc32: 0x%08x", checksum_crc32);
    go_idle(false);
  }

  /* -*- send helpers -*- */

  /**
   * @brief Go to IDLE mode
   *
   * @param is_error_ mark that caused in error case
   * @param r_errno_ set r_errno in error case
   */
  void go_idle(bool is_error_, int r_errno_ = 0)
  {
    op_state = OP::IDLE;
    is_error = is_error_;
    if (is_error && r_errno_ != 0) {
      r_errno = r_errno_;
    } else if (!is_error) {
      r_errno = 0;
    }
    cond.notify_all();
  }

  void send_reset()
  {
    RCLCPP_DEBUG(get_logger(), "FTP:m: kCmdResetSessions");
    if (!session_file_map.empty()) {
      RCLCPP_WARN(
        get_logger(), "FTP: Reset closes %zu sessions",
        session_file_map.size());
      session_file_map.clear();
    }

    op_state = OP::ACK;
    FTPRequest req(FTPRequest::kCmdResetSessions);
    req.send(uas, ++last_send_seqnr);
  }

  /// Send any command with string payload (usually file/dir path)
  inline void send_any_path_command(
    const FTPRequest::Opcode op, const std::string & debug_msg,
    const std::string & path, const uint32_t offset)
  {
    RCLCPP_DEBUG_STREAM(get_logger(), "FTP:m: " << debug_msg << path << " off: " << offset);
    FTPRequest req(op);
    auto hdr = req.header();
    hdr.offset = offset;
    req.set_header(hdr);
    req.set_data_string(path);
    req.send(uas, ++last_send_seqnr);
  }

  void send_list_command()
  {
    send_any_path_command(
      FTPRequest::kCmdListDirectory, "kCmdListDirectory: ", list_path,
      list_offset);
  }

  void send_open_ro_command()
  {
    send_any_path_command(FTPRequest::kCmdOpenFileRO, "kCmdOpenFileRO: ", open_path, 0);
  }

  void send_open_wo_command()
  {
    send_any_path_command(FTPRequest::kCmdOpenFileWO, "kCmdOpenFileWO: ", open_path, 0);
  }

  void send_create_command()
  {
    send_any_path_command(FTPRequest::kCmdCreateFile, "kCmdCreateFile: ", open_path, 0);
  }

  void send_terminate_command(uint32_t session)
  {
    RCLCPP_DEBUG_STREAM(get_logger(), "FTP:m: kCmdTerminateSession: " << session);
    FTPRequest req(FTPRequest::kCmdTerminateSession, session);
    auto hdr = req.header();
    hdr.offset = 0;
    hdr.size = 0;
    req.set_header(hdr);
    req.send(uas, ++last_send_seqnr);
  }

  void send_read_command()
  {
    const auto bytes_to_read = std::min<size_t>(
      read_size - read_buffer.size(),
      FTPRequest::DATA_MAXSZ);
    RCLCPP_DEBUG_STREAM(
      get_logger(), "FTP:m: kCmdReadFile: " << active_session << " off: " << read_offset <<
        " sz: " << bytes_to_read);
    FTPRequest req(FTPRequest::kCmdReadFile, active_session);
    auto hdr = req.header();
    hdr.offset = read_offset;
    hdr.size = bytes_to_read;
    req.set_header(hdr);
    req.send(uas, ++last_send_seqnr);
  }

  void send_burst_read_command()
  {
    RCLCPP_DEBUG_STREAM(
      get_logger(), "FTP:m: kCmdBurstReadFile: " << active_session << " off: " << read_offset);
    FTPRequest req(FTPRequest::kCmdBurstReadFile, active_session);
    auto hdr = req.header();
    hdr.offset = read_offset;
    hdr.size = FTPRequest::DATA_MAXSZ;
    req.set_header(hdr);
    // Each burst gets its own seqnr window, so packets still streaming from an
    // earlier burst cannot be taken for this one.
    last_send_seqnr += BURST_SEQ_WINDOW;
    req.send(uas, ++last_send_seqnr);
  }

  void send_write_command(const size_t bytes_to_copy)
  {
    // write chunk from write_buffer [write_it..bytes_to_copy]
    RCLCPP_DEBUG_STREAM(
      get_logger(), "FTP:m: kCmdWriteFile: " << active_session << " off: " << write_offset <<
        " sz: " << bytes_to_copy);
    FTPRequest req(FTPRequest::kCmdWriteFile, active_session);
    auto hdr = req.header();
    hdr.offset = write_offset;
    hdr.size = bytes_to_copy;
    req.set_header(hdr);
    std::copy(write_it, write_it + bytes_to_copy, req.data());
    write_chunk_size = bytes_to_copy;
    req.send(uas, ++last_send_seqnr);
  }

  void send_remove_command(const std::string & path)
  {
    send_any_path_command(FTPRequest::kCmdRemoveFile, "kCmdRemoveFile: ", path, 0);
  }

  bool send_rename_command(const std::string & old_path, const std::string & new_path)
  {
    std::ostringstream os;
    os << old_path;
    os << '\0';
    os << new_path;

    std::string paths = os.str();
    if (paths.size() >= FTPRequest::DATA_MAXSZ) {
      RCLCPP_ERROR(get_logger(), "FTP: rename file paths is too long: %zu", paths.size());
      r_errno = ENAMETOOLONG;
      return false;
    }

    send_any_path_command(FTPRequest::kCmdRename, "kCmdRename: ", paths, 0);
    return true;
  }

  void send_truncate_command(const std::string & path, size_t length)
  {
    send_any_path_command(FTPRequest::kCmdTruncateFile, "kCmdTruncateFile: ", path, length);
  }

  void send_create_dir_command(const std::string & path)
  {
    send_any_path_command(FTPRequest::kCmdCreateDirectory, "kCmdCreateDirectory: ", path, 0);
  }

  void send_remove_dir_command(const std::string & path)
  {
    send_any_path_command(FTPRequest::kCmdRemoveDirectory, "kCmdRemoveDirectory: ", path, 0);
  }

  void send_calc_file_crc32_command(const std::string & path)
  {
    send_any_path_command(FTPRequest::kCmdCalcFileCRC32, "kCmdCalcFileCRC32: ", path, 0);
  }

  /* -*- helpers -*- */

  bool add_dirent(const char * ptr, size_t slen)
  {
    mavros_msgs::msg::FileEntry ent;
    ent.size = 0;

    if (ptr[0] == FTPRequest::DIRENT_DIR) {
      ent.name.assign(ptr + 1, slen - 1);
      ent.type = mavros_msgs::msg::FileEntry::TYPE_DIRECTORY;

      RCLCPP_DEBUG_STREAM(get_logger(), "FTP:List Dir: " << ent.name);
    } else {
      // ptr[0] == FTPRequest::DIRENT_FILE
      std::string name_size(ptr + 1, slen - 1);

      auto sep_it = std::find(name_size.begin(), name_size.end(), '\t');
      ent.name.assign(name_size.begin(), sep_it);
      ent.type = mavros_msgs::msg::FileEntry::TYPE_FILE;

      if (sep_it != name_size.end()) {
        name_size.erase(name_size.begin(), sep_it + 1);
        if (name_size.size() != 0) {
          try {
            ent.size = std::stoi(name_size);
          } catch (const std::logic_error &) {
            return false;
          }
        }
      }

      RCLCPP_DEBUG_STREAM(get_logger(), "FTP:List File: " << ent.name << " SZ: " << ent.size);
    }

    list_entries.push_back(ent);
    return true;
  }

  void list_directory_end()
  {
    RCLCPP_DEBUG(get_logger(), "FTP:List done");
    go_idle(false);
  }

  void list_directory(const std::string & path)
  {
    list_offset = 0;
    list_path = path;
    list_entries.clear();
    op_state = OP::LIST;

    send_list_command();
  }

  bool open_file(const std::string & path, int mode)
  {
    open_path = path;
    open_size = 0;
    op_state = OP::OPEN;

    if (mode == mavros_msgs::srv::FileOpen::Request::MODE_READ) {
      send_open_ro_command();
    } else if (mode == mavros_msgs::srv::FileOpen::Request::MODE_WRITE) {
      send_open_wo_command();
    } else if (mode == mavros_msgs::srv::FileOpen::Request::MODE_CREATE) {
      send_create_command();
    } else {
      RCLCPP_ERROR(get_logger(), "FTP: Unsupported open mode: %d", mode);
      op_state = OP::IDLE;
      r_errno = EINVAL;
      return false;
    }

    return true;
  }

  bool close_file(const std::string & path)
  {
    auto it = session_file_map.find(path);
    if (it == session_file_map.end()) {
      RCLCPP_ERROR(get_logger(), "FTP:Close %s: not opened", path.c_str());
      r_errno = EBADF;
      return false;
    }

    op_state = OP::ACK;
    send_terminate_command(it->second.session);
    session_file_map.erase(it);
    return true;
  }

  void read_file_end()
  {
    RCLCPP_DEBUG(get_logger(), "FTP:Read done");
    go_idle(false);
  }

  bool read_file(const std::string & path, size_t off, size_t len)
  {
    auto it = session_file_map.find(path);
    if (it == session_file_map.end()) {
      RCLCPP_ERROR(get_logger(), "FTP:Read %s: not opened", path.c_str());
      r_errno = EBADF;
      return false;
    }

    op_state = OP::READ;
    active_session = it->second.session;
    read_size = len;
    read_offset = off;
    read_buffer.clear();
    if (read_buffer.capacity() < len ||
      read_buffer.capacity() > len + MAX_RESERVE_DIFF)
    {
      // reserve memory
      read_buffer.reserve(len);
    }

    // A burst streams until EOF or the FCU's own limit (ArduPilot 500 or 2000
    // packets, PX4 ~35 KB) and cannot be stopped. Use it only when the read runs
    // to EOF, so nothing past the requested range is sent.
    read_burst = burst_supported && off + len >= it->second.size;
    burst_resyncs = 0;
    if (read_burst) {
      send_burst_read_command();
    } else {
      send_read_command();
    }
    return true;
  }

  void write_file_end()
  {
    RCLCPP_DEBUG(get_logger(), "FTP:Write done");
    go_idle(false);
  }

  bool write_file(const std::string & path, size_t off, V_FileData & data)
  {
    auto it = session_file_map.find(path);
    if (it == session_file_map.end()) {
      RCLCPP_ERROR(get_logger(), "FTP:Write %s: not opened", path.c_str());
      r_errno = EBADF;
      return false;
    }

    op_state = OP::WRITE;
    active_session = it->second.session;
    write_offset = off;
    write_buffer = std::move(data);
    write_it = write_buffer.begin();

    send_write_command(write_bytes_to_copy());
    return true;
  }

  void remove_file(const std::string & path)
  {
    op_state = OP::ACK;
    send_remove_command(path);
  }

  bool rename_(const std::string & old_path, const std::string & new_path)
  {
    op_state = OP::ACK;
    return send_rename_command(old_path, new_path);
  }

  void truncate_file(const std::string & path, size_t length)
  {
    op_state = OP::ACK;
    send_truncate_command(path, length);
  }

  void create_directory(const std::string & path)
  {
    op_state = OP::ACK;
    send_create_dir_command(path);
  }

  void remove_directory(const std::string & path)
  {
    op_state = OP::ACK;
    send_remove_dir_command(path);
  }

  void checksum_crc32_file(const std::string & path)
  {
    op_state = OP::CHECKSUM;
    checksum_crc32 = 0;
    send_calc_file_crc32_command(path);
  }

  size_t write_bytes_to_copy()
  {
    return std::min<size_t>(
      std::distance(write_it, write_buffer.end()),
      FTPRequest::DATA_MAXSZ);
  }

  /**
   * Wait for go_idle(). Caller holds @a lock since the request was sent,
   * so a fast reply cannot be missed. Times out after @a msecs without any reply.
   */
  bool wait_completion(std::unique_lock<std::mutex> & lock, const int msecs)
  {
    const auto timeout = std::chrono::milliseconds(msecs);
    uint32_t last_progress = rx_progress;

    while (op_state != OP::IDLE) {
      if (cond.wait_for(lock, timeout) == std::cv_status::timeout &&
        op_state != OP::IDLE && rx_progress == last_progress)
      {
        op_state = OP::IDLE;
        r_errno = ETIMEDOUT;
        return false;
      }
      last_progress = rx_progress;
    }

    return !is_error;
  }

  /* -*- service callbacks -*- */

  /**
   * Service handler common header code.
   */
#define SERVICE_IDLE_CHECK() \
  std::unique_lock<std::mutex> lock(cond_mutex); \
  if (op_state != OP::IDLE) { \
    RCLCPP_ERROR(get_logger(), "FTP: Busy"); \
    res->success = false; \
    res->r_errno = EBUSY; \
    return; \
  }

  void list_cb(
    const mavros_msgs::srv::FileList::Request::SharedPtr req,
    mavros_msgs::srv::FileList::Response::SharedPtr res)
  {
    SERVICE_IDLE_CHECK();

    list_directory(req->dir_path);
    res->success = wait_completion(lock, LIST_TIMEOUT_MS);
    res->r_errno = r_errno;
    if (res->success) {
      res->list = std::move(list_entries);
      list_entries.clear();                     // not sure that it's needed
    }
  }

  void open_cb(
    const mavros_msgs::srv::FileOpen::Request::SharedPtr req,
    mavros_msgs::srv::FileOpen::Response::SharedPtr res)
  {
    SERVICE_IDLE_CHECK();

    // only one session per file
    auto it = session_file_map.find(req->file_path);
    if (it != session_file_map.end()) {
      RCLCPP_ERROR(
        get_logger(), "FTP: File %s: already opened",
        req->file_path.c_str());
      res->success = false;
      res->r_errno = EALREADY;
      return;
    }

    res->success = open_file(req->file_path, req->mode);
    if (res->success) {
      res->success = wait_completion(lock, OPEN_TIMEOUT_MS);
      res->size = open_size;
    }
    res->r_errno = r_errno;
  }

  void close_cb(
    const mavros_msgs::srv::FileClose::Request::SharedPtr req,
    mavros_msgs::srv::FileClose::Response::SharedPtr res)
  {
    SERVICE_IDLE_CHECK();

    res->success = close_file(req->file_path);
    if (res->success) {
      res->success = wait_completion(lock, OPEN_TIMEOUT_MS);
    }
    res->r_errno = r_errno;
  }

  void read_cb(
    const mavros_msgs::srv::FileRead::Request::SharedPtr req,
    mavros_msgs::srv::FileRead::Response::SharedPtr res)
  {
    SERVICE_IDLE_CHECK();

    res->success = read_file(req->file_path, req->offset, req->size);
    if (res->success) {
      res->success = wait_completion(lock, CHUNK_TIMEOUT_MS);
    }
    if (res->success) {
      res->data = std::move(read_buffer);
      read_buffer.clear();                      // same as for list_entries
    }
    res->r_errno = r_errno;
  }

  void write_cb(
    const mavros_msgs::srv::FileWrite::Request::SharedPtr req,
    mavros_msgs::srv::FileWrite::Response::SharedPtr res)
  {
    SERVICE_IDLE_CHECK();

    res->success = write_file(req->file_path, req->offset, req->data);
    if (res->success) {
      res->success = wait_completion(lock, CHUNK_TIMEOUT_MS);
    }
    write_buffer.clear();
    res->r_errno = r_errno;
  }

  void remove_cb(
    const mavros_msgs::srv::FileRemove::Request::SharedPtr req,
    mavros_msgs::srv::FileRemove::Response::SharedPtr res)
  {
    SERVICE_IDLE_CHECK();

    remove_file(req->file_path);
    res->success = wait_completion(lock, OPEN_TIMEOUT_MS);
    res->r_errno = r_errno;
  }

  void rename_cb(
    const mavros_msgs::srv::FileRename::Request::SharedPtr req,
    mavros_msgs::srv::FileRename::Response::SharedPtr res)
  {
    SERVICE_IDLE_CHECK();

    res->success = rename_(req->old_path, req->new_path);
    if (res->success) {
      res->success = wait_completion(lock, OPEN_TIMEOUT_MS);
    }
    res->r_errno = r_errno;
  }

  void truncate_cb(
    const mavros_msgs::srv::FileTruncate::Request::SharedPtr req,
    mavros_msgs::srv::FileTruncate::Response::SharedPtr res)
  {
    SERVICE_IDLE_CHECK();

    // Note: emulated truncate() can take a while
    truncate_file(req->file_path, req->length);
    res->success = wait_completion(lock, LIST_TIMEOUT_MS * 5);
    res->r_errno = r_errno;
  }

  void mkdir_cb(
    const mavros_msgs::srv::FileMakeDir::Request::SharedPtr req,
    mavros_msgs::srv::FileMakeDir::Response::SharedPtr res)
  {
    SERVICE_IDLE_CHECK();

    create_directory(req->dir_path);
    res->success = wait_completion(lock, OPEN_TIMEOUT_MS);
    res->r_errno = r_errno;
  }

  void rmdir_cb(
    const mavros_msgs::srv::FileRemoveDir::Request::SharedPtr req,
    mavros_msgs::srv::FileRemoveDir::Response::SharedPtr res)
  {
    SERVICE_IDLE_CHECK();

    remove_directory(req->dir_path);
    res->success = wait_completion(lock, OPEN_TIMEOUT_MS);
    res->r_errno = r_errno;
  }

  void checksum_cb(
    const mavros_msgs::srv::FileChecksum::Request::SharedPtr req,
    mavros_msgs::srv::FileChecksum::Response::SharedPtr res)
  {
    SERVICE_IDLE_CHECK();

    checksum_crc32_file(req->file_path);
    res->success = wait_completion(lock, LIST_TIMEOUT_MS);
    res->crc32 = checksum_crc32;
    res->r_errno = r_errno;
  }

#undef SERVICE_IDLE_CHECK

  /**
   * @brief Reset communication on both sides.
   * @note This call break other calls, so use carefully.
   */
  void reset_cb(
    const std_srvs::srv::Empty::Request::SharedPtr req [[maybe_unused]],
    std_srvs::srv::Empty::Response::SharedPtr res [[maybe_unused]])
  {
    std::unique_lock<std::mutex> lock(cond_mutex);
    send_reset();
    session_file_map.clear();
    wait_completion(lock, OPEN_TIMEOUT_MS);
  }
};

}       // namespace std_plugins
}       // namespace mavros

#include <mavros/mavros_plugin_register_macro.hpp>  // NOLINT
MAVROS_PLUGIN_REGISTER(mavros::std_plugins::FTPPlugin)
