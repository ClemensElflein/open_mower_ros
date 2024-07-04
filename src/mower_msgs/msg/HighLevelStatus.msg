uint8 HIGH_LEVEL_STATE_NULL=0
uint8 HIGH_LEVEL_STATE_IDLE=1
uint8 HIGH_LEVEL_STATE_AUTONOMOUS=2
uint8 HIGH_LEVEL_STATE_RECORDING=3

uint8 SUBSTATE_1=0
uint8 SUBSTATE_2=1
uint8 SUBSTATE_3=2
uint8 SUBSTATE_4=3
uint8 SUBSTATE_SHIFT=6

uint8 state
string state_name
string sub_state_name
int16 current_area
int16 current_path
int16 current_path_index
float32 gps_quality_percent
float32 battery_percent
bool is_charging
bool emergency
