#!/usr/bin/env bash
docker run --rm -it \
  -v "$(pwd)/ws:/home/dev/ws" \
  -w /home/dev/ws" \
  t37-sim:jazzy bash
