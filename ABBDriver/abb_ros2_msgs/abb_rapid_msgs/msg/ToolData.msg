#-------------------------------------------------------------------------------
# Description:
#   The purpose of this message definition, is to represent the
#   RAPID data type "tooldata" (tool data).
#
# Note:
#   Please see the "Technical reference manual - RAPID Instructions,
#   Functions and Data types" for more details.
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# Message fields
#-------------------------------------------------------------------------------
# Defines whether or not the robot is holding the tool:
# - TRUE: The robot is holding the tool.
# - FALSE: The robot is not holding the tool, that is, a stationary tool.
bool robhold

# The tool coordinate system, that is:
# - The position of the TCP (x, y and z) in mm,
#   expressed in the wrist coordinate system (tool0).
# - The orientation of the tool coordinate system,
#   expressed in the wrist coordinate system.
abb_rapid_msgs/Pose tframe

# The load of the tool (if robot held tool).
# The load of the gripper holding the work object (if stationary tool).
abb_rapid_msgs/LoadData tload
