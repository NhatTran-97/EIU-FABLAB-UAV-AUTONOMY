#!/bin/bash
export DISPLAY=:0
xhost +local:root  

sudo docker run -it \
    --rm \
    --runtime=nvidia --gpus all \
    --name fablab_container \
    -e LANG=C.UTF-8 \
    -e LC_ALL=C.UTF-8  \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --network=host \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/usr/bin/tegrastats:/usr/bin/tegrastats" \
    --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
    -v /dev:/dev \
    -v /tmp/argus_socket:/tmp/argus_socket \
    --privileged \
    --volume="/home/fablab01/drone_ws:/home/drone_ws" \
    fablab_drone_vision_01:latest
