# Publishes the current state of the kalman filter, intended for debug purposes

float64 x
float64 y
float64 theta
float64 vx
float64 vr

# Filter uncertainty, as standard deviations (m, m, rad)
float64 sigma_x
float64 sigma_y
float64 sigma_theta

# Innovation of the most recent GPS position update (m), and its normalised
# form. The NIS is the honest check on the noise model: with a 2 DOF
# measurement it should average ~2.0. Consistently above means the filter is
# over-confident (R too tight or Q too small); below means the opposite.
float64 position_innovation_x
float64 position_innovation_y
float64 position_nis
