# One measured EXECUTE_PATH interval. Percent values are percentage points
# (0..100), converted from sensor_msgs/BatteryState.percentage (0..1).
std_msgs/Header header
uint64 sample_id
geometry_msgs/PoseStamped start_pose
geometry_msgs/PoseStamped end_pose
# Accumulated three-dimensional distance travelled, not endpoint displacement.
float64 distance_m
float64 start_battery_percent
float64 end_battery_percent
float64 consumed_percent
float64 percentage_per_meter
# False for incomplete/stale telemetry or unusable consumption/distance data.
bool valid_for_estimate
string outcome
