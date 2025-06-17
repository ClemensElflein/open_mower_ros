uint8 MOWER_STATUS_INITIALIZING=0
uint8 MOWER_STATUS_OK=255



time stamp
# Low level stuff
# Initialized or not
uint8 mower_status
bool raspberry_pi_power
bool is_charging
bool esc_power
bool rain_detected
bool sound_module_available
bool sound_module_busy
bool ui_board_available
bool mow_enabled

# ESC stuff
uint8   mower_esc_status
float32 mower_esc_temperature
float32 mower_esc_current
float32 mower_motor_temperature
float32 mower_motor_rpm
