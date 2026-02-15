# ftp

- File: `mavros/src/plugins/ftp.cpp`
- Class: `mavros::std_plugins::FTPPlugin`
- Namespace: `ftp`
- Brief: FTP plugin.


## Publishers
- None


## Subscribers
- None


## Services
- `~/list` (mavros_msgs::srv::FileList)
- `~/open` (mavros_msgs::srv::FileOpen)
- `~/close` (mavros_msgs::srv::FileClose)
- `~/read` (mavros_msgs::srv::FileRead)
- `~/write` (mavros_msgs::srv::FileWrite)
- `~/mkdir` (mavros_msgs::srv::FileMakeDir)
- `~/rmdir` (mavros_msgs::srv::FileRemoveDir)
- `~/remove` (mavros_msgs::srv::FileRemove)
- `~/truncate` (mavros_msgs::srv::FileTruncate)
- `~/reset` (std_srvs::srv::Empty)
- `~/rename` (mavros_msgs::srv::FileRename)
- `~/checksum` (mavros_msgs::srv::FileChecksum)


## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- [`FILE_TRANSFER_PROTOCOL`](https://mavlink.io/en/messages/common.html#FILE_TRANSFER_PROTOCOL) [handler: handle_file_transfer_protocol, dialect: common, msg_id: 110, id: `mavlink::common::msg::FILE_TRANSFER_PROTOCOL::MSG_ID`]


## MAVLink Publications
- [`FILE_TRANSFER_PROTOCOL`](https://mavlink.io/en/messages/common.html#FILE_TRANSFER_PROTOCOL) [arg: `this`, dialect: common, msg_id: 110, id: `mavlink::common::msg::FILE_TRANSFER_PROTOCOL::MSG_ID`]
