# Vedder VESC open source motor controller state (telemetry)

# fault codes
int32 FAULT_CODE_NONE=0
int32 FAULT_CODE_OVER_VOLTAGE=1
int32 FAULT_CODE_UNDER_VOLTAGE=2
int32 FAULT_CODE_DRV8302=3
int32 FAULT_CODE_ABS_OVER_CURRENT=4
int32 FAULT_CODE_OVER_TEMP_FET=5
int32 FAULT_CODE_OVER_TEMP_MOTOR=6

float64 voltage_input        # input voltage (volt)
float64 temperature_pcb      # temperature of printed circuit board (degrees Celsius)
float64 current_motor        # motor current (ampere)
float64 current_input        # input current (ampere)
float64 speed                # motor electrical speed (revolutions per minute) 
float64 duty_cycle           # duty cycle (0 to 1)
float64 charge_drawn         # electric charge drawn from input (ampere-hour)
float64 charge_regen         # electric charge regenerated to input (ampere-hour)
float64 energy_drawn         # energy drawn from input (watt-hour)
float64 energy_regen         # energy regenerated to input (watt-hour)
float64 displacement         # net tachometer (counts)
float64 distance_traveled    # total tachnometer (counts)
int32   fault_code
