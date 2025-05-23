#-------------------------------------------------------------------------------
# Description:
#   The purpose of this message definition, is to represent the
#   RAPID data type "wobjdata" (work object data).
#
# Note:
#   Please see the "Technical reference manual - RAPID Instructions,
#   Functions and Data types" for more details.
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# Message fields
#-------------------------------------------------------------------------------
# Defines whether or not the robot in the actual
# program task is holding the work object:
# - TRUE: The robot is holding the work object,
#         i.e. using a stationary tool.
# - FALSE: The robot is not holding the work object,
#          i.e. the robot is holding the tool.
bool robhold

# Defines whether or not a fixed user coordinate system is used:
# - TRUE: Fixed user coordinate system.
# - FALSE: Movable user coordinate system,
#          i.e. coordinated external axes are used.
#          Also to be used in a MultiMove system in
#          semicoordinated or synchronized coordinated mode.
bool ufprog

# The mechanical unit with which the robot movements are coordinated.
#
# Only specified in the case of movable user
# coordinate systems (ufprog is FALSE).
string ufmec

# The user coordinate system, i.e. the position
# of the current work surface or fixture.
abb_rapid_msgs/Pose uframe

# The object coordinate system, i.e. the position
# of the current work object.
abb_rapid_msgs/Pose oframe
