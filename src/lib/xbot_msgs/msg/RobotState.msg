# A state to display to a human to tell what's going on
float32 battery_percentage
bool emergency
bool is_charging
bool rain_detected
float32 gps_percentage
float32 current_action_progress
string current_state                # Like "Cleaning"
string current_sub_state            # Like "doing the dishes"
int16 current_area
int16 current_path
int16 current_path_index
xbot_msgs/AbsolutePose robot_pose # Pose of the docking station