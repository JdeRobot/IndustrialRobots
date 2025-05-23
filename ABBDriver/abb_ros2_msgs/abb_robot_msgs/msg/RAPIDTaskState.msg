#-------------------------------------------------------------------------------
# Description:
#   The purpose of this message definition, is to represent a subset of states
#   of a RAPID task defined in an ABB robot controller system.
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# Enumerations
#-------------------------------------------------------------------------------
# RAPID execution states:
uint8 EXECUTION_STATE_UNKNOWN       = 1
uint8 EXECUTION_STATE_READY         = 2
uint8 EXECUTION_STATE_STOPPED       = 3
uint8 EXECUTION_STATE_STARTED       = 4
uint8 EXECUTION_STATE_UNINITIALIZED = 5

#-------------------------------------------------------------------------------
# Message fields
#-------------------------------------------------------------------------------
# The RAPID task's name.
string name

# The RAPID task's activated/deactivated status.
bool activated

# The RAPID task's execution state.
uint8 execution_state

# The RAPID task is a motion task.
bool motion_task
