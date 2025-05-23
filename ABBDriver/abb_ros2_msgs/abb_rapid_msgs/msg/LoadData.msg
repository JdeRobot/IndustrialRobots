#-------------------------------------------------------------------------------
# Description:
#   The purpose of this message definition, is to represent the
#   RAPID data type "loaddata" (load data).
#
# Note:
#   Please see the "Technical reference manual - RAPID Instructions,
#   Functions and Data types" for more details.
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# Message fields
#-------------------------------------------------------------------------------
# The mass (weight) of the load in kg.
float32 mass

# The center of gravity of the payload, expressed in mm,
# in the tool coordinate system if the robot is holding the tool.
#
# If a stationary tool is used then the center of gravity for the payload
# held by the gripper is expressed in the object frame of the work object
# coordinate system moved by the robot.
abb_rapid_msgs/Pos cog

# The orientation of the axes of moment.
#
# These are the principal axes of the payload
# moment of inertia with origin in cog.
#
# If the robot is holding the tool, the axes of moment
# are expressed in the tool coordinate system.
abb_rapid_msgs/Orient aom

# The moment of inertia of the load around the x-, y- and z-axis,
# expressed in kgm^2.
float32 ix
float32 iy
float32 iz
