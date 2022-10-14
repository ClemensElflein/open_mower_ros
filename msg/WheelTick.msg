# This message is used to count the wheel ticks for our robot.
# It supports up to four wheels, if there are only two in use, use WHEEL_RL and WHEEL_RR
# Ticks should always count independent of the wheel rotation. Use the direction to show wheel spin rotation

uint8   WHEEL_VALID_FL=1
uint8   WHEEL_VALID_FR=2
uint8   WHEEL_VALID_RL=4
uint8   WHEEL_VALID_RR=8

time stamp
uint32 wheel_tick_factor    # factor to use to convert wheel ticks to m
uint8  valid_wheels         # Bitfield to show which ticks are valid
uint8 wheel_direction_fl    # wheel direction front left
uint32 wheel_ticks_fl       # wheel ticks front left
uint8 wheel_direction_fr    # wheel direction front right
uint32 wheel_ticks_fr       # wheel ticks front right
uint8 wheel_direction_rl    # wheel direction rear left
uint32 wheel_ticks_rl       # wheel ticks rear left
uint8 wheel_direction_rr    # wheel direction rear right
uint32 wheel_ticks_rr       # wheel ticks rear right
