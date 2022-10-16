# This message is used to describe the absolute pose measured by some sensor.
# Usually the data will be noisy and might optionally include an orientation estimate.
# We can assume that this position is drift free.

# an enum showing from which type of absolute sensor the data originates
uint8 SOURCE_GPS=1
uint8 SOURCE_LIGHTHOUSE=2

# Some flags to describe the GPS fix better
uint16 FLAG_GPS_RTK=1                   # True, if there are corrections applied
uint16 FLAG_GPS_RTK_FIXED=2             # True, if we have a fixed solution
uint16 FLAG_GPS_RTK_FLOAT=4             # True, if we have a float solution
uint16 FLAG_GPS_DEAD_RECKONING=8        # True, if the GPS module is currently doing dead reckoning using its internal IMU


std_msgs/Header header
uint32 sensor_stamp                     # A timestamp sent by the sensor
uint32 received_stamp                   # A timestamp taken on receive

uint8 source
uint16 flags                            # Flags specific to the source
uint8 orientation_valid                 # Set to true, if the sensor can also measure orientation
uint8 motion_vector_valid               # Set to true, if the speed vector is valid
float32 position_accuracy               # Set to the estimated accuracy in m
float32 orientation_accuracy            # Set to the estimated orientation accuracy in rad
geometry_msgs/PoseWithCovariance pose   # The pose measured by the sensor
geometry_msgs/Vector3 motion_vector     # Vector of motion in m/s
float64 vehicle_heading                 # The yaw for the orientation
float64 motion_heading                  # The yaw for the motion vector


