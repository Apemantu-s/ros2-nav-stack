#!/bin/bash
xhost +local:docker > /dev/null 2>&1
docker run -it --rm --name ros \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$(cd "$(dirname "$0")/.." && pwd)":/ws \
  ros2-nav
