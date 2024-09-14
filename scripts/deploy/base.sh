#!/bin/bash

run_docker() {
    xhost +local:root # Giving display privileges

    # -it is for interactive, tty
    # --privileged for accessing /dev contents
    # --net=host to share the same network as host machine. TL;DR same IP.
    docker run -it --privileged --net=host \
    --name limo_bot \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    -v $(dirname "$0")/app.sh:/root/app.sh \
    -v $(dirname "$0")/../../workspace/limo_control/launch/limo_control.launch.py:/root/limo_control.launch.py \
    -v $(dirname "$0")/../../workspace/limo_control/logs/generate_error_plot.py:/root/generate_error_plot.py \
    -v $(dirname "$0")/../../workspace/src/target_error.csv:/root/target_error.csv \
    $@
}

stop_docker() {
    docker stop limo_bot && docker rm limo_bot
    xhost -local:root # Remove display privileges
}