# Vedder VESC open source motor controller telemetry

# fault codes
int32 FAULT_CODE_NONE=0
int32 FAULT_CODE_OVER_VOLTAGE=1
int32 FAULT_CODE_UNDER_VOLTAGE=2
int32 FAULT_CODE_DRV8302=3
int32 FAULT_CODE_ABS_OVER_CURRENT=4
int32 FAULT_CODE_OVER_TEMP_FET=5
int32 FAULT_CODE_OVER_TEMP_MOTOR=6

Header  header
float32 voltage_input        # input voltage (volt)
float32 temperature_pcb      # temperature of printed circuit board (degrees Celsius)
float32 current_motor        # motor current (ampere)
float32 current_input        # input current (ampere)
float32 speed                # motor electrical speed (revolutions per minute) 
float32 duty_cycle           # duty cycle (0 to 1)
float32 charge_drawn         # electric charge drawn from input (ampere-hour)
float32 charge_regen         # electric charge regenerated to input (ampere-hour)
float32 energy_drawn         # energy drawn from input (watt-hour)
float32 energy_regen         # energy regenerated to input (watt-hour)
int32   displacement         # net tachometer (counts)
int32   distance_traveled    # total tachnometer (counts)
int32   fault_code
