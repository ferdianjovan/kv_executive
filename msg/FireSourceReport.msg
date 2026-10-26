# Confirmed fire observation viewpoint; this is not a ranged fire coordinate.
std_msgs/Header header
uint64 site_id
string label
geometry_msgs/PoseStamped local_pose
geometry_msgs/PoseStamped planning_pose
sensor_msgs/NavSatFix global_position
uint32 source_count
uint64 track_id
float32 exclusion_radius
float32 bearing_rad
