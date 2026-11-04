# Inspection result. Target positions are estimates distinct from the observer.
std_msgs/Header header
uint64 site_id
bool fire_found
uint32 inspection_attempts
string status
uint32 source_count
uint64 track_id

geometry_msgs/PoseStamped observer_local_pose
geometry_msgs/PoseStamped observer_planning_pose
sensor_msgs/NavSatFix observer_global_position
bool observer_global_valid

# Local/map validity describes an associated source estimate, not semantic ground truth.
bool smoke_location_valid
bool smoke_global_valid
geometry_msgs/PointStamped smoke_local_position
geometry_msgs/PointStamped smoke_planning_position
sensor_msgs/NavSatFix smoke_global_position
# Geometric source-localization uncertainty; classification comes from camera detections.
float64 smoke_uncertainty_m
string smoke_location_method

# Historical qualified smoke-wall measurement retained when arrival was confirmed.
# The point headers retain measurement time; these are not newly ranged coordinates.
bool smoke_wall_confirmed
float64 smoke_wall_nearest_depth_m
string smoke_wall_evidence

bool fire_location_valid
bool fire_global_valid
geometry_msgs/PointStamped fire_local_position
geometry_msgs/PointStamped fire_planning_position
sensor_msgs/NavSatFix fire_global_position
# Geometric source-localization uncertainty; classification comes from camera detections.
float64 fire_uncertainty_m
string fire_location_method

# Fresh compressed camera evidence. Missing streams do not erase a valid finding.
InspectionImageEvidence rgb_evidence
InspectionImageEvidence thermal_evidence
bool evidence_complete
string evidence_status
