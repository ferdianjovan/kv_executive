# Live straight-line return estimate, expressed in battery percentage points.
std_msgs/Header header
geometry_msgs/PoseStamped current_pose
geometry_msgs/PoseStamped home_pose
float64 remaining_percent
float64 home_distance_m
float64 mean_percent_per_meter
float64 return_required_percent
# return_required_percent * return_safety_factor + reserve_percent.
float64 buffered_required_percent
# remaining_percent - buffered_required_percent.
float64 return_margin_percent
# Check availability before interpreting the numeric estimate fields.
bool estimate_available
bool return_alert
# Number of usable samples contributing to the model, including the active
# EXECUTE_PATH interval once it has sufficient distance and positive depletion.
uint64 sample_count
