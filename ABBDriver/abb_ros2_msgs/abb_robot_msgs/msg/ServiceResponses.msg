# result codes are 16 bit unsigned

# Generic OK
string SUCCESS=""
uint16 RC_SUCCESS=1

# Generic failure message
string FAILED="Service failed (e.g. timeout or resource not found, see the DEBUG log for details)"
uint16 RC_FAILED=2

# The server's underlying communication interface is busy
string SERVER_IS_BUSY="Server is busy"
uint16 RC_SERVER_IS_BUSY=1001

# Filename field is empty in service request
string EMPTY_FILENAME="Filename is empty"
uint16 RC_EMPTY_FILENAME=2001

# Signal name field is empty in service request
string EMPTY_SIGNAL_NAME="Signal name is empty"
uint16 RC_EMPTY_SIGNAL_NAME=2002

# RAPID task name field is empty in service request
string EMPTY_RAPID_TASK_NAME="RAPID task name is empty"
uint16 RC_EMPTY_RAPID_TASK_NAME=2003

# RAPID module name field is empty in service request
string EMPTY_RAPID_MODULE_NAME="RAPID module name is empty"
uint16 RC_EMPTY_RAPID_MODULE_NAME=2004

# RAPID symbol name field is empty in service request
string EMPTY_RAPID_SYMBOL_NAME="RAPID symbol name is empty"
uint16 RC_EMPTY_RAPID_SYMBOL_NAME=2005

# Robot controller is not in auto mode
string NOT_IN_AUTO_MODE="Robot controller is not in AUTO mode"
uint16 RC_NOT_IN_AUTO_MODE=3001

# The motors are off
string MOTORS_ARE_OFF="Motors are off"
uint16 RC_MOTORS_ARE_OFF=3002

# The motors are on
string MOTORS_ARE_ON="Motors are on"
uint16 RC_MOTORS_ARE_ON=3003

# RAPID has not been stopped
string RAPID_NOT_STOPPED="RAPID has not been stopped"
uint16 RC_RAPID_NOT_STOPPED=3004

# RAPID is not running
string RAPID_NOT_RUNNING="RAPID is not running"
uint16 RC_RAPID_NOT_RUNNING=3005

# RobotWare StateMachine Add-In runtime state is missing
string SM_RUNTIME_STATES_MISSING="No runtime states received for any StateMachine Add-In instance"
uint16 RC_SM_RUNTIME_STATES_MISSING=4001

# RobotWare StateMachine Add-In-instances do not know RAPID task
string SM_UNKNOWN_RAPID_TASK="RAPID task not found among known StateMachine Add-In instances"
uint16 RC_SM_UNKNOWN_RAPID_TASK=4002

# RobotWare StateMachine Add-In instance has not been initialized
string SM_UNINITIALIZED="StateMachine Add-In instance has not been initialized"
uint16 RC_SM_UNINITIALIZED=4003
