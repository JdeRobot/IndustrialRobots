# This message holds specific data available from Neobotix RelayBoardV2

std_msgs/Header header

# Possible Communication States
int16 CS_NOT_ESTABLISHED = 0 		# configuration process has not been started
int16 CS_CONFIGURATION_FAILED = 1	# configuration process failed
int16 CS_OK = 2 			# communication is OK
int16 CS_LOST = 3 			# comminication was lost
int16 CS_ERROR = 4			# comminication has errors

# Possible Charging States
int16 CHS_NOTCHARGING = 0 		# no charging in progress; can be started with service /RelayBoardV2/StartCharging
int16 CHS_START_CHARGING = 1		# RelayBoardV2 is starting charging process
int16 CHS_CHARGING = 2 			# charging in progress; can be stopped with service /RelayBoardV2/StopCharging
int16 CHS_NO_CHARGER = 3 		# charging has been started but no charger was connected; this aborts charging
int16 CHS_BRAKES_OPEN = 4 		# brakes have been opened while charging; this aborts charging
int16 CHS_EMSTOP = 5 			# EMSTOP has been issued while charging; this aborts charging
int16 CHS_ABORT = 6 	   		# charging abort by host system
int16 CHS_FINISHED = 7 			# charging finished

#
bool[16] relayboardv2_state		# Shows state for all possible errors
					# relayboardv2_state[0] if true no errors; if false check other fields
					# relayboardv2_state[1] if true charging relay error; check robot hardware
					# relayboardv2_state[2] if true release brakes button failed; check robot hardware
					# relayboardv2_state[3] if true motor error; check robot hardware
					# relayboardv2_state[4] if true safety relay error; check robot hardware
					# relayboardv2_state[5] if true Leistungsrelais error; check robot hardware
					# relayboardv2_state[6] if true EMStop system error; check robot hardware
					# relayboardv2_state[7] reserved
					# relayboardv2_state[8] reserved
					# relayboardv2_state[9] reserved
					# relayboardv2_state[10] reserved
					# relayboardv2_state[11] reserved
					# relayboardv2_state[12] reserved
					# relayboardv2_state[13] reserved
					# relayboardv2_state[14] reserved
					# relayboardv2_state[15] reserved


int16 communication_state		# values see above

int16 charging_state			# values see above

int16 temperature			# temperature on relayboardv2 [°C]

float32 battery_voltage			# battery voltage [V]

float32 charging_current		# charging current [A] if not charging = 0

bool[4] relay_states			# state for each relay on RelayBoardV2; True = Active; False = Inactive
					# relay_states[0] is our charging relay
					# other are free to use

bool[8] keypad				# state for each key on keypad; True = Pressed
					# keypad[0] info button
					# keypad[1] home button
					# keypad[2] start button
					# keypad[3] stop button
					# keypad[4] release brakes button
					# keypad[5] on demand digital input
					# keypad[6] on demand digital input
					# keypad[7] on demand digital input

bool shutdown				# if true host system is asked to shut down


