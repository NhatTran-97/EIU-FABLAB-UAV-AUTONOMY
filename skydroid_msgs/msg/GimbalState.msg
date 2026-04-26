std_msgs/Header header

# Connection
bool is_connected
string firmware_version

# Attitude
float32 yaw_deg
float32 pitch_deg
float32 roll_deg

uint8 control_mode   # GimbalCommand.MANUAL=0 | GimbalCommand.AUTO=1
