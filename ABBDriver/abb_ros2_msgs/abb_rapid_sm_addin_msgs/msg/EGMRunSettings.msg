#-------------------------------------------------------------------------------
# Description:
#   The purpose of this message definition, is to represent the custom RAPID
#   data type "EGMRunSettings" from the RobotWare "StateMachine Add-In".
#
# Note:
#   "EGMRunSettings" are for example used with the RAPID EGM instructions
#   "EGMRunJoint" and "EGMRunPose".
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# Message fields
#-------------------------------------------------------------------------------
# Condition time [s] for how long an EGM motion's convergence criteria must be
# fulfilled before the motion is considered to be finished.
float32 cond_time

# Ramp in time [s].
float32 ramp_in_time

# Static offset applied on top of the EGM references
# (only used in EGM pose mode).
abb_rapid_msgs/Pose offset

# Position correction gain for the EGM controller.
float32 pos_corr_gain
