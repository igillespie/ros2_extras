# PanTiltState.msg
std_msgs/Header header # Add this field for timestamp and frame_id
float64 pan_position  # Current pan position in degrees
float64 tilt_position # Current tilt position in degrees
string status         # Status of the controller (e.g., "OK", "ERROR")
