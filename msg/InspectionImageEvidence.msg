# Compressed original camera evidence. Invalid images carry a reason in status.
bool valid
string status
sensor_msgs/CompressedImage image
string original_encoding
uint32 original_width
uint32 original_height
uint32 stored_width
uint32 stored_height

# Source-time pose; false if odometry did not bracket the image acquisition.
bool observer_pose_valid
geometry_msgs/PoseStamped observer_local_pose

# Metadata is valid only when its source image matches image.header exactly.
bool detection_valid
std_msgs/Header detection_header
uint8 detection_status
bool detected
uint64 track_id
uint32 source_count
uint32 bbox_x
uint32 bbox_y
uint32 bbox_width
uint32 bbox_height
bool angular_bounds_valid
float32 yaw_min_rad
float32 yaw_max_rad
float32 vertical_min_rad
float32 vertical_max_rad
