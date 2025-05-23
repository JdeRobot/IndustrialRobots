#-------------------------------------------------------------------------------
# Description:
#   The purpose of this message definition, is to represent the
#   RAPID data type "pose" (coordinate transformations).
#
# Note:
#   Please see the "Technical reference manual - RAPID Instructions,
#   Functions and Data types" for more details.
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# Message fields
#-------------------------------------------------------------------------------
# The displacement in position (x, y, and z) of the coordinate system.
abb_rapid_msgs/Pos trans

# The rotation of the coordinate system.
abb_rapid_msgs/Orient rot
