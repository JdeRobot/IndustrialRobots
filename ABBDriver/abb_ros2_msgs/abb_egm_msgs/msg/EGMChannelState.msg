#-------------------------------------------------------------------------------
# Description:
#   The purpose of this message definition, is to represent a subset of states
#   of an EGM communication channel. I.e. a channel between an EGM client,
#   in an ABB robot controller system, and an external EGM server.
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# Enumerations
#-------------------------------------------------------------------------------
# EGM client states:
uint8 EGM_UNDEFINED = 1
uint8 EGM_ERROR     = 2
uint8 EGM_STOPPED   = 3
uint8 EGM_RUNNING   = 4

# Motor states:
uint8 MOTORS_UNDEFINED = 1
uint8 MOTORS_ON        = 2
uint8 MOTORS_OFF       = 3

# RAPID states:
uint8 RAPID_UNDEFINED = 1
uint8 RAPID_STOPPED   = 2
uint8 RAPID_RUNNING   = 3

#-------------------------------------------------------------------------------
# Message fields
#-------------------------------------------------------------------------------
# The EGM channel's name.
string name

# The EGM channel's active/inactive status.
bool active

# Indicator for if the EGM convergence criteria is currently fulfilled or not.
bool egm_convergence_met

# EGM client state.
uint8 egm_client_state

# Motor state.
uint8 motor_state

# RAPID execution state.
uint8 rapid_execution_state

# Utilization rate of the connected mechanical unit.
#
# An utilization rate > 100 mean that EGM motion references are
# too aggressive and should be reduced.
float64 utilization_rate
