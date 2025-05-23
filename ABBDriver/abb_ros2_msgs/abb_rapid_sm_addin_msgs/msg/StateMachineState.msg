#-------------------------------------------------------------------------------
# Description:
#   The purpose of this message definition, is to represent the state of an
#   instance of the RobotWare "StateMachine Add-In"'s RAPID implementation.
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# Enumerations
#-------------------------------------------------------------------------------
# Possible "StateMachine Add-In" RAPID states:
#
# Note: These enumeration values differ from the RAPID implementation
#       values to avoid conflict with default values in ROS messages.
uint8 SM_STATE_UNKNOWN           = 1
uint8 SM_STATE_IDLE              = 2
uint8 SM_STATE_INITIALIZE        = 3
uint8 SM_STATE_RUN_RAPID_ROUTINE = 4
uint8 SM_STATE_RUN_EGM_ROUTINE   = 5

# Possible "StateMachine Add-In" RAPID EGM actions:
#
# Note: These enumeration values differ from the RAPID implementation
#       values to avoid conflict with default values in ROS messages.
uint8 EGM_ACTION_UNKNOWN      = 1
uint8 EGM_ACTION_NONE         = 2
uint8 EGM_ACTION_RUN_JOINT    = 3
uint8 EGM_ACTION_RUN_POSE     = 4
uint8 EGM_ACTION_STOP         = 5
uint8 EGM_ACTION_START_STREAM = 6
uint8 EGM_ACTION_STOP_STREAM  = 7

#-------------------------------------------------------------------------------
# Message fields
#-------------------------------------------------------------------------------
# The RAPID task that runs the "StateMachine Add-In" instance.
string rapid_task

# The "StateMachine Add-In" instance's current state.
uint8 sm_state

# The "StateMachine Add-In" instance's current EGM action
# (only used if the RobotWare EGM option is present).
uint8 egm_action
