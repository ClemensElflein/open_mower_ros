# Static device info of the battery pack. Published once (latched) on "/ll/bms/info" after
# the low level answered the GetDeviceInfo RPC; it does not change while the pack is connected.
time stamp
string mfr_name
string mfr_date
string dev_name
string dev_version
string dev_chemistry
uint16 serial_number
uint8 cell_count
float32 design_voltage_v
float32 design_capacity_ah
