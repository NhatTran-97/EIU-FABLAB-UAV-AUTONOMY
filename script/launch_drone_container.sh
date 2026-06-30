#!/bin/bash
# Interactive version — opens the drone container and drops you into a bash shell
# (for manual debugging). For unattended boot use launch_drone_bringup.sh instead.
export DISPLAY=:0
xhost +local:root

log_info()  { echo -e "ℹ️  $1"; }
log_ok()    { echo -e "✅ $1"; }
log_fail()  { echo -e "❌ $1" >&2; return 1; }

LORA_SERIAL="0001"

log_info "Scanning for LORA device..."
LORA_PORT=$(for dev in /dev/ttyUSB*; do
    [[ -e "$dev" ]] || continue
    udevadm info --query=all --name="$dev" | grep -q "ID_SERIAL_SHORT=$LORA_SERIAL" && echo "$dev" && break
done)

if [ -z "$LORA_PORT" ]; then
    log_fail "LORA device with serial $LORA_SERIAL not found"
    exit 1
else
    log_ok "LORA found at $LORA_PORT"
fi

# Camera is optional — don't fail just because it isn't plugged in.
DEVICE_ARGS=( --device="$LORA_PORT:/dev/lora_drone" )
[[ -e /dev/video0 ]] && DEVICE_ARGS+=( --device=/dev/video0 )

# Drop a stale container from a previous unclean exit (fixed --name would clash).
sudo docker rm -f fablab_drone_container 2>/dev/null || true

sudo docker run -it \
    --rm \
    --name fablab_drone_container \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="ROS_DOMAIN_ID=7" \
    --network=host \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    "${DEVICE_ARGS[@]}" \
    --privileged \
    --volume="/home/fablab01/drone_ws:/home/drone_ws" \
    fablab_drone_01:latest \
    /bin/bash -c "source /home/drone_ws/install/setup.bash && exec bash"
