"""Simple motor model for closed-loop blade_speed_adapter replay tests.

Maps (grass_thickness, robot_speed) -> (blade_rpm, blade_current), reproducing
the two dominant non-linearities seen in the real sensor logs
(data/sensorlog_20260420-*.csv):

1. Linear RPM droop: more load -> lower blade RPM.
2. Non-monotonic current: rises during load onset, then COLLAPSES at full stall
   (xESC input-current limiter + duty backoff, see
   .hypertask/task/blade_speed_adapter_analysis.md section 2.3).

Load is proportional to cutting rate, i.e. grass_thickness * robot_speed.
At zero robot speed there is no cutting, so RPM returns to nominal and current
to idle regardless of grass.
"""

# Calibrated to the observed log envelope
SPEED_MAX = 0.4         # m/s; speed at which the log was recorded
RPM_NOMINAL = 4400.0    # unloaded spin
RPM_STALL = 500.0       # near-full-stall floor
CURRENT_IDLE = 0.4      # A, baseline idle-spin current
CURRENT_PEAK = 1.5      # A, ceiling imposed by l_in_current_max
STALL_LOAD = 0.75       # load fraction at which current starts to collapse
CURRENT_STALL = 0.5     # A, steady-state current when fully bogged


def _clip01(x):
    if x < 0.0:
        return 0.0
    if x > 1.0:
        return 1.0
    return x


def _steady_state(grass_thickness, robot_speed):
    """Instantaneous, lossless (rpm, current) the motor *would* settle to."""
    load = _clip01(grass_thickness * (robot_speed / SPEED_MAX))
    rpm = RPM_NOMINAL - load * (RPM_NOMINAL - RPM_STALL)
    if load < STALL_LOAD:
        current = CURRENT_IDLE + load * (CURRENT_PEAK - CURRENT_IDLE) / STALL_LOAD
    else:
        collapse = (load - STALL_LOAD) / (1.0 - STALL_LOAD)
        current = CURRENT_PEAK - collapse * (CURRENT_PEAK - CURRENT_STALL)
    return rpm, current


class MotorModel:
    """Stateful motor model with first-order rotor/current inertia.

    Without lag the closed-loop `speed -> rpm -> sag -> speed` feedback has
    gain > 1 at high grass loads and oscillates. A real blade has rotor
    inertia, so rpm can't change instantaneously — the lag damps the loop.

    Default time constants are rough (sub-second), matching the
    analysis's "recovery is sharp ~1 s" observation.
    """

    def __init__(self, dt=0.1, tau_rpm=0.3, tau_current=0.15):
        self.rpm = RPM_NOMINAL
        self.current = CURRENT_IDLE
        self.alpha_rpm = dt / (tau_rpm + dt)
        self.alpha_current = dt / (tau_current + dt)

    def step(self, grass_thickness, robot_speed):
        target_rpm, target_current = _steady_state(grass_thickness, robot_speed)
        self.rpm += (target_rpm - self.rpm) * self.alpha_rpm
        self.current += (target_current - self.current) * self.alpha_current
        return self.rpm, self.current


# Back-compat shim (existing callers pass instantaneous values).
def synthesize(grass_thickness, robot_speed):
    return _steady_state(grass_thickness, robot_speed)
