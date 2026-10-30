# Scope an asynchronous parent-state request to one accepted ExecuteMission goal.
# The executive ignores requests whose UUID does not match the current goal.
unique_identifier_msgs/UUID goal_id
# Verify interrupts Exploration and enters verification within the active goal.
# Abort ends the action with LOITER recovery. Return remains unimplemented
# and also aborts to LOITER.
string state
