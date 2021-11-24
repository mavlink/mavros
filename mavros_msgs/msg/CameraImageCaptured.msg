# MAVLink message: CAMERA_IMAGE_CAPTURED
# https://mavlink.io/en/messages/common.html#CAMERA_IMAGE_CAPTURED

std_msgs/Header header

geometry_msgs/Quaternion orientation	# Quaternion of camera orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
geographic_msgs/GeoPoint geo
float32 relative_alt	# mm	Altitude above ground
int32 image_index # Zero based index of this image (i.e. a new image will have index CAMERA_CAPTURE_STATUS.image count -1)
int8 capture_result # Boolean indicating success (1) or failure (0) while capturing this image.
string file_url #URL of image taken. Either local storage or http://foo.jpg if camera provides an HTTP interface.
