#-------------------------------------------------------------------------------
# Description:
#   The purpose of this message definition, is to represent the custom RAPID
#   data type "EGMSettings" from the RobotWare "StateMachine Add-In".
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# Message fields
#-------------------------------------------------------------------------------
# Indicator for if EGM motions are allowed to start.
bool allow_egm_motions

# Indicator for if RAPID tasks should be synced before starting EGM motions
# (only used in multi robot systems).
bool use_presync

# Settings for EGMSetupUC instructions.
abb_rapid_sm_addin_msgs/EGMSetupSettings setup_uc

# Settings for EGMAct instructions.
abb_rapid_sm_addin_msgs/EGMActivateSettings activate

# Settings for EGMRun instructions.
abb_rapid_sm_addin_msgs/EGMRunSettings run

# Settings for EGMStop instructions.
abb_rapid_sm_addin_msgs/EGMStopSettings stop
