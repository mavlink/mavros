# ftp

- File: `mavros/src/plugins/ftp.cpp`
- Class: `mavros::std_plugins::FTPPlugin`
- Namespace: `ftp`
- Brief: FTP plugin.


Implements the [MAVLink File Transfer Protocol](https://mavlink.io/en/services/ftp.html).

## Publishers
- None


## Subscribers
- None


## Services
- `~/list` ([mavros_msgs::srv::FileList](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/FileList.html)) - List the contents of a directory on the FCU (FTP).
- `~/open` ([mavros_msgs::srv::FileOpen](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/FileOpen.html)) - Open a file on the FCU for reading or writing (FTP).
- `~/close` ([mavros_msgs::srv::FileClose](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/FileClose.html)) - Close an open file on the FCU (FTP).
- `~/read` ([mavros_msgs::srv::FileRead](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/FileRead.html)) - Read data from an open file on the FCU (FTP).
- `~/write` ([mavros_msgs::srv::FileWrite](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/FileWrite.html)) - Write data to an open file on the FCU (FTP).
- `~/mkdir` ([mavros_msgs::srv::FileMakeDir](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/FileMakeDir.html)) - Create a directory on the FCU (FTP).
- `~/rmdir` ([mavros_msgs::srv::FileRemoveDir](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/FileRemoveDir.html)) - Remove a directory on the FCU (FTP).
- `~/remove` ([mavros_msgs::srv::FileRemove](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/FileRemove.html)) - Remove a file on the FCU (FTP).
- `~/truncate` ([mavros_msgs::srv::FileTruncate](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/FileTruncate.html)) - Truncate a file to a given length on the FCU (FTP).
- `~/reset` ([std_srvs::srv::Empty](https://docs.ros.org/en/rolling/p/std_srvs/srv/Empty.html)) - Reset the FTP session on both sides (FTP).
- `~/rename` ([mavros_msgs::srv::FileRename](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/FileRename.html)) - Rename a file on the FCU (FTP).
- `~/checksum` ([mavros_msgs::srv::FileChecksum](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/FileChecksum.html)) - Calculate the CRC32 checksum of a file on the FCU (FTP).


## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- [`FILE_TRANSFER_PROTOCOL`](https://mavlink.io/en/messages/common.html#FILE_TRANSFER_PROTOCOL) [handler: handle_file_transfer_protocol, dialect: common, msg_id: 110, id: `mavlink::common::msg::FILE_TRANSFER_PROTOCOL::MSG_ID`]


## MAVLink Publications
- [`FILE_TRANSFER_PROTOCOL`](https://mavlink.io/en/messages/common.html#FILE_TRANSFER_PROTOCOL) [arg: `this`, dialect: common, msg_id: 110, id: `mavlink::common::msg::FILE_TRANSFER_PROTOCOL::MSG_ID`]
