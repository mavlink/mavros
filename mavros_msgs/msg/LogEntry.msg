# Information about a single log
#
#  :id: - log id
#  :num_logs: - total number of logs
#  :last_log_num: - id of last log
#  :time_utc: - UTC timestamp of log (ros::Time(0) if not available)
#  :size: - size of log in bytes (may be approximate)

std_msgs/Header header

uint16 id
uint16 num_logs
uint16 last_log_num
time time_utc
uint32 size
