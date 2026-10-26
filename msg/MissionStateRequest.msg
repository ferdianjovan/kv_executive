# Scope an asynchronous parent-state request to one accepted ExecuteMission goal.
# The executive ignores requests whose UUID does not match the current goal.
unique_identifier_msgs/UUID goal_id
# Abort stops Exploration and ends the action with LOITER recovery.
# Legacy Return/Verify requests also stop the action; those states are dormant.
string state
