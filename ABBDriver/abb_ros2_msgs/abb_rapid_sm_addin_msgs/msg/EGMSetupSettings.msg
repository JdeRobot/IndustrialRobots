#-------------------------------------------------------------------------------
# Description:
#   The purpose of this message definition, is to represent the custom RAPID
#   data type "EGMSetupSettings" from the RobotWare "StateMachine Add-In".
#
# Note:
#   "EGMSetupSettings" are for example used with the RAPID EGM instruction
#   "EGMSetupUC".
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# Message fields
#-------------------------------------------------------------------------------
# Indicator for if the EGM controller should apply filtering on the
# EGM references.
bool use_filtering

# Communication timeout [s].
float32 comm_timeout
