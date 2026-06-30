#!/bin/bash
# Headless/boot version of launch_drone_container.sh — runs the ROS2 bringup
# directly (no interactive shell) so systemd can supervise it.
set -u

export DISPLAY=:0
xhost +local:root 2>/dev/null || true   # non-fatal when headless / no X yet

LORA_SERIAL="0001"

# Wait for the LoRa USB device to enumerate (USB can be slow right after boot).
echo "ℹ️  Waiting for LORA device (serial $LORA_SERIAL)..."
LORA_PORT=""
for i in $(seq 1 30); do
    for dev in /dev/ttyUSB*; do
        [[ -e "$dev" ]] || continue
        if udevadm info --query=all --name="$dev" 2>/dev/null \
             | grep -q "ID_SERIAL_SHORT=$LORA_SERIAL"; then
            LORA_PORT="$dev"; break
        fi
    done
    [[ -n "$LORA_PORT" ]] && break
    sleep 2
done

if [[ -z "$LORA_PORT" ]]; then
    echo "❌ LORA device serial $LORA_SERIAL not found after 60s" >&2
    exit 1
fi
echo "✅ LORA found at $LORA_PORT"

# Camera is optional — don't fail the whole drone if it isn't plugged in.
DEVICE_ARGS=( --device="$LORA_PORT:/dev/lora_drone" )
[[ -e /dev/video0 ]] && DEVICE_ARGS+=( --device=/dev/video0 )

# Remove a stale container from a previous unclean exit, then run in the
# FOREGROUND (exec) so systemd tracks it and forwards SIGTERM for clean stop.
docker rm -f fablab_drone_container 2>/dev/null || true

exec docker run --rm \
    --name fablab_drone_container \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="ROS_DOMAIN_ID=7" \
    --env="PYTHONUNBUFFERED=1" \
    --env="RCUTILS_LOGGING_BUFFERED_STREAM=0" \
    --network=host \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    "${DEVICE_ARGS[@]}" \
    --privileged \
    --volume="/home/fablab01/drone_ws:/home/drone_ws" \
    fablab_drone_01:latest \
    /bin/bash -c "source /home/drone_ws/install/setup.bash && exec ros2 launch lora_drone_package bringup.launch.py"
