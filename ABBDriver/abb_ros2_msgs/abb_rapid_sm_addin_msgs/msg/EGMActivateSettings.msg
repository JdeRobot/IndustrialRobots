#-------------------------------------------------------------------------------
# Description:
#   The purpose of this message definition, is to represent the custom RAPID
#   data type "EGMActivateSettings" from the RobotWare "StateMachine Add-In".
#
# Note:
#   "EGMActivateSettings" are for example used with the RAPID EGM instructions
#   "EGMActJoint" and "EGMActPose".
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# Message fields
#-------------------------------------------------------------------------------
# Tool to use for EGM motions.
abb_rapid_msgs/ToolData tool

# Work object to use for EGM motions.
abb_rapid_msgs/WObjData wobj

# Correction frame to use for EGM motions (only used in EGM pose mode).
abb_rapid_msgs/Pose correction_frame

# Sensor frame to use for EGM motions (only used in EGM pose mode).
abb_rapid_msgs/Pose sensor_frame

# Condition threshold value [deg or mm] for when an EGM motion's convergence
# criteria is considered to be fulfilled.
float32 cond_min_max

# Low pass filter bandwidth [Hz] for the EGM controller.
float32 lp_filter

# Sample rate [ms] for the EGM communication.
uint8 sample_rate

# Maximum admitted joint speed [deg/s].
float32 max_speed_deviation
