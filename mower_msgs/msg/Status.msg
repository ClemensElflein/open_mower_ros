uint8 MOWER_STATUS_INITIALIZING=0
uint8 MOWER_STATUS_OK=255

uint8 ESC_STATUS_ERROR=0
uint8 ESC_STATUS_STALLED=1
uint8 ESC_STATUS_RUNNING=254
uint8 ESC_STATUS_OK=255

time stamp
uint8 mower_status
float32 temperature_mow
float32 v_charge
float32 v_battery
uint8 left_esc_status
uint8 right_esc_status
uint8 mow_esc_status