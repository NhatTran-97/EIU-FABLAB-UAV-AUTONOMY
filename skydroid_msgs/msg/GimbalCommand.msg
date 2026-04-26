# Gimbal control command

# --- Control mode (ai đang gửi lệnh này) ---
uint8 MANUAL=0
uint8 AUTO=1
uint8 control_mode   # MANUAL=0 | AUTO=1

# --- Gimbal mode ---
uint8 MODE_PTZ=0
uint8 MODE_POSITION=1
uint8 MODE_VELOCITY=2
uint8 mode

# ---- PTZ mode (chỉ dùng khi mode=MODE_PTZ) ----
uint8 PTZ_STOP=0
uint8 PTZ_UP=1
uint8 PTZ_DOWN=2
uint8 PTZ_CENTER=3
uint8 PTZ_FOLLOW=4
uint8 PTZ_LOCK=5

uint8 ptz_cmd

# ---- Position mode (chỉ dùng khi mode=MODE_POSITION) ----
float32 pitch_deg
float32 yaw_deg
float32 pitch_speed_dps
float32 yaw_speed_dps
bool enable_pitch
bool enable_yaw

# ---- Velocity mode (chỉ dùng khi mode=MODE_VELOCITY) ----
float32 pitch_vel_dps
float32 yaw_vel_dps
