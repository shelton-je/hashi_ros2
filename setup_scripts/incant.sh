#!/bin/bash
xhost +local:hashi && docker run --gpus all --ipc=host --network host --ulimit memlock=-1 --ulimit stack=67108864 -it --rm \
-v /tmp/.X11-unix:/tmp/.X11-unix -v /home/$USER/projects:/projects -v /home/$USER/colcon_ws/src:/colcon_ws/src -e DISPLAY=unix$DISPLAY --workdir="/colcon_ws/src" \
-i hashi
