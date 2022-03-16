uint8 MOWER_STATUS_INITIALIZING=0
uint8 MOWER_STATUS_OK=255



time stamp
# Low level stuff
# Initialized or not
uint8 mower_status
bool raspberry_pi_power
bool gps_power
bool esc_power
bool rain_detected
bool sound_module_available
bool sound_module_busy
bool ui_board_available
float32[5] ultrasonic_ranges
bool emergency
float32 v_charge
float32 v_battery
float32 charge_current

# ESC stuff
ESCStatus left_esc_status
ESCStatus right_esc_status
ESCStatus mow_esc_status

