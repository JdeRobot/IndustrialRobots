#-------------------------------------------------------------------------------
# Description:
#   The purpose of this message definition, is to represent a subset of states
#   of an ABB robot controller system.
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# Message fields
#-------------------------------------------------------------------------------
# Metadata.
std_msgs/Header header

# Motors on/off indicator.
bool motors_on

# Auto/manual mode indicator.
bool auto_mode

# RAPID running/stopped indicator.
bool rapid_running

# RAPID task states.
abb_robot_msgs/RAPIDTaskState[] rapid_tasks

# Mechanical unit states.
abb_robot_msgs/MechanicalUnitState[] mechanical_units
