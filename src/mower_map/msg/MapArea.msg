string name
geometry_msgs/Polygon area
geometry_msgs/Polygon[] obstacles
int32 outline_count          # per-area override; -1 means use the global config value
int32 outline_overlap_count  # per-area override; -1 means use the global config value
float64 outline_offset       # per-area override; -1.0 means use the global config value
