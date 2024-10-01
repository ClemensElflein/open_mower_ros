# an enum showing from which type of data the sensor delivers
uint8 TYPE_STRING=1
uint8 TYPE_DOUBLE=2

uint8 VALUE_DESCRIPTION_UNKNOWN=0
uint8 VALUE_DESCRIPTION_TEMPERATURE=1
uint8 VALUE_DESCRIPTION_VELOCITY=2
uint8 VALUE_DESCRIPTION_ACCELERATION=3
uint8 VALUE_DESCRIPTION_VOLTAGE=4
uint8 VALUE_DESCRIPTION_CURRENT=5
uint8 VALUE_DESCRIPTION_PERCENT=6
uint8 VALUE_DESCRIPTION_DISTANCE=7
uint8 VALUE_DESCRIPTION_RPM=8

# Some flags to describe the GPS fix better
uint16 FLAG_GPS_RTK=1                   # True, if there are corrections applied
uint16 FLAG_GPS_RTK_FIXED=2             # True, if we have a fixed solution
uint16 FLAG_GPS_RTK_FLOAT=4             # True, if we have a float solution
uint16 FLAG_GPS_DEAD_RECKONING=8        # True, if the GPS module is currently doing dead reckoning using its internal IMU

uint16 FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE=1    # True, if we have a recent absolute position measurement (e.g. RTK GPS)
uint16 FLAG_SENSOR_FUSION_DEAD_RECKONING=8          # True, if we have dead reckoning

string sensor_id
string sensor_name                      # A human readable name for this sensor
uint8 value_type                        # Data type of sensor data to expect. Useful for parsing
uint8 value_description                 # Description of the value (e.g. Temperature, Speed), may be used to display different kinds of widgets to the user
string unit                             # Unit of the sensor data (e.g. "V" or "m/s")
bool has_min_max                        # True, if min_value and max_value are valid
float64 min_value                       # Absolute sensor min (e.g. used for rendering a gauge)
float64 max_value                       # Absolute sensor max (e.g. used for rendering a gauge)
bool has_critical_low                   # True, if there's a lower bound that's considered critical and a user should be warned about
float64 lower_critical_value            # Any value below this is considered critical
bool has_critical_high                  # True, if there's an upper bound that's considered critical
float64 upper_critical_value            # Any value above this is considered critical
