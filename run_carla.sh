#!/bin/bash
docker pull carlasim/carla:0.9.10

EXTRA_ARGS="-it"
if [[ "$DETACH" ]]; then
  EXTRA_ARGS="-d"
fi

docker kill carla_sim || true
docker run \
  --name carla_sim \
  --rm \
  --gpus all \
  --net=host \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -e SDL_VIDEODRIVER=offscreen\
  $EXTRA_ARGS \
  carlasim/carla:0.9.10 \
  /bin/bash  ./CarlaUE4.sh -opengl -quality-level=Low #-nosound -benchmark -fps=20 