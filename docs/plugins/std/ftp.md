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
- `~/list` ([mavros_msgs::srv::FileList](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/FileList.html))
- `~/open` ([mavros_msgs::srv::FileOpen](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/FileOpen.html))
- `~/close` ([mavros_msgs::srv::FileClose](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/FileClose.html))
- `~/read` ([mavros_msgs::srv::FileRead](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/FileRead.html))
- `~/write` ([mavros_msgs::srv::FileWrite](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/FileWrite.html))
- `~/mkdir` ([mavros_msgs::srv::FileMakeDir](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/FileMakeDir.html))
- `~/rmdir` ([mavros_msgs::srv::FileRemoveDir](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/FileRemoveDir.html))
- `~/remove` ([mavros_msgs::srv::FileRemove](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/FileRemove.html))
- `~/truncate` ([mavros_msgs::srv::FileTruncate](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/FileTruncate.html))
- `~/reset` ([std_srvs::srv::Empty](https://docs.ros.org/en/rolling/p/std_srvs/srv/Empty.html))
- `~/rename` ([mavros_msgs::srv::FileRename](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/FileRename.html))
- `~/checksum` ([mavros_msgs::srv::FileChecksum](https://docs.ros.org/en/rolling/p/mavros_msgs/srv/FileChecksum.html))


## Clients
- None


## Parameters
- None


## MAVLink Subscriptions
- [`FILE_TRANSFER_PROTOCOL`](https://mavlink.io/en/messages/common.html#FILE_TRANSFER_PROTOCOL) [handler: handle_file_transfer_protocol, dialect: common, msg_id: 110, id: `mavlink::common::msg::FILE_TRANSFER_PROTOCOL::MSG_ID`]


## MAVLink Publications
- [`FILE_TRANSFER_PROTOCOL`](https://mavlink.io/en/messages/common.html#FILE_TRANSFER_PROTOCOL) [arg: `this`, dialect: common, msg_id: 110, id: `mavlink::common::msg::FILE_TRANSFER_PROTOCOL::MSG_ID`]
