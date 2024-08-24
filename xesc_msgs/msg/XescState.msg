# fault codes
uint32 XESC_FAULT_UNINITIALIZED=1
uint32 XESC_FAULT_WATCHDOG=2
uint32 XESC_FAULT_UNDERVOLTAGE=4
uint32 XESC_FAULT_OVERVOLTAGE=8
uint32 XESC_FAULT_OVERCURRENT=16
uint32 XESC_FAULT_OVERTEMP_MOTOR=32
uint32 XESC_FAULT_OVERTEMP_PCB=64
uint32 XESC_FAULT_INVALID_HALL=128
uint32 XESC_FAULT_INTERNAL_ERROR=256
uint32 XESC_FAULT_OPEN_LOAD=512

# connection states
uint8 XESC_CONNECTION_STATE_DISCONNECTED=0
uint8 XESC_CONNECTION_STATE_WAITING_FOR_FW=1
uint8 XESC_CONNECTION_STATE_CONNECTED_INCOMPATIBLE_FW=2
uint8 XESC_CONNECTION_STATE_CONNECTED=3

uint8   connection_state
uint8   fw_major
uint8   fw_minor
float64 voltage_input        # input voltage (volt)
float64 temperature_pcb      # temperature of printed circuit board (degrees Celsius)
float64 temperature_motor    # temperature of connected motor (degrees Celsius)
float64 current_input        # input current (ampere)
float64 duty_cycle           # duty cycle (0 to 1)
uint8   direction            # current direction
uint32  tacho                # wheel ticks
uint32  tacho_absolute       # wheel ticks absolute
int16   rpm                  # revolutions per minute (of the axis/shaft)
uint32  fault_code
