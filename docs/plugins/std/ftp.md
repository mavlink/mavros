# ftp

- File: `mavros/src/plugins/ftp.cpp`
- Class: `mavros::std_plugins::FTPPlugin`
- Namespace: `mavros::std_plugins`
- Brief: FTP plugin.


Implements the
[MAVLink File Transfer Protocol](https://mavlink.io/en/services/ftp.html).

## Publishers
- None

## Subscribers
- None

## Services
- `~/list` [type: [mavros_msgs::srv::FileList](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/FileList.html)] - List the contents of a directory on the FCU (FTP).
- `~/open` [type: [mavros_msgs::srv::FileOpen](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/FileOpen.html)] - Open a file on the FCU for reading or writing (FTP).
- `~/close` [type: [mavros_msgs::srv::FileClose](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/FileClose.html)] - Close an open file on the FCU (FTP).
- `~/read` [type: [mavros_msgs::srv::FileRead](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/FileRead.html)] - Read data from an open file on the FCU (FTP).
- `~/write` [type: [mavros_msgs::srv::FileWrite](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/FileWrite.html)] - Write data to an open file on the FCU (FTP).
- `~/mkdir` [type: [mavros_msgs::srv::FileMakeDir](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/FileMakeDir.html)] - Create a directory on the FCU (FTP).
- `~/rmdir` [type: [mavros_msgs::srv::FileRemoveDir](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/FileRemoveDir.html)] - Remove a directory on the FCU (FTP).
- `~/remove` [type: [mavros_msgs::srv::FileRemove](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/FileRemove.html)] - Remove a file on the FCU (FTP).
- `~/truncate` [type: [mavros_msgs::srv::FileTruncate](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/FileTruncate.html)] - Truncate a file to a given length on the FCU (FTP).
- `~/reset` [type: [std_srvs::srv::Empty](https://docs.ros.org/en/rolling/p/std_srvs/srv/Empty.html)] - Reset the FTP session on both sides (FTP).
- `~/rename` [type: [mavros_msgs::srv::FileRename](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/FileRename.html)] - Rename a file on the FCU (FTP).
- `~/checksum` [type: [mavros_msgs::srv::FileChecksum](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/FileChecksum.html)] - Calculate the CRC32 checksum of a file on the FCU (FTP).

## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- [`FILE_TRANSFER_PROTOCOL`](https://mavlink.io/en/messages/common.html#FILE_TRANSFER_PROTOCOL) [handler: handle_file_transfer_protocol, dialect: common, msg_id: 110, id: `mavlink::common::msg::FILE_TRANSFER_PROTOCOL::MSG_ID`] - handler for mavlink::common::msg::FILE_TRANSFER_PROTOCOL


## MAVLink Publications
- [`FILE_TRANSFER_PROTOCOL`](https://mavlink.io/en/messages/common.html#FILE_TRANSFER_PROTOCOL) [arg: `msg`, dialect: common, msg_id: 110, id: `mavlink::common::msg::FILE_TRANSFER_PROTOCOL::MSG_ID`]
