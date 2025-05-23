#-------------------------------------------------------------------------------
# Description:
#   The purpose of this message definition, is to represent the
#   RAPID data type "jointtarget" (joint position data).
#
# Note:
#   Please see the "Technical reference manual - RAPID Instructions,
#   Functions and Data types" for more details.
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# Message fields
#-------------------------------------------------------------------------------
# Axis positions of the robot axes in degrees.
abb_rapid_msgs/RobJoint robax

# The position of the external axes.
abb_rapid_msgs/ExtJoint extax
