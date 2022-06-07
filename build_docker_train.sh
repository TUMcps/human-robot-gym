#! /bin/bash
# Run this file either with ./build_docker.sh user or ./build_docker.sh root. 
# User mode will create a user in docker so that the files you create are not made by root.
# Root mode creates a "classic" root user in docker. Use that power at your own risk.
docker build \
-f Dockerfile.train \
--build-arg MODE=root \
-t human-robot-gym-train/$USER:v1 .