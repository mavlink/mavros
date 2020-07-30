# Reply to LogRequestData, - a chunk of a log
#
#  :id: - log id
#  :offset: - offset into the log
#  :data: - chunk of data (if zero-sized, then there are no more chunks)

std_msgs/Header header

uint16 id
uint32 offset
uint8[] data
