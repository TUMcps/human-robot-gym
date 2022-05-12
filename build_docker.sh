#! /bin/bash
# Run this file either with ./build_docker.sh user or ./build_docker.sh root. 
# User mode will create a user in docker so that the files you create are not made by root.
# Root mode creates a "classic" root user in docker. Use that power at your own risk.
user=${1:-user}
echo "Chosen user mode"=$user
if [ "$user" = "root" ]
then
  docker build \
    -f Dockerfile.dev \
    --build-arg MODE=root \
    -t human-robot-gym/$USER:v1 .
elif [ "$user" = "user" ]
then
  docker build \
    -f Dockerfile.dev \
    --build-arg MODE=user \
    --build-arg USER_UID=$(id -u) \
    --build-arg USER_GID=$(id -g) \
    --build-arg USERNAME=$USER \
    -t human-robot-gym/$USER:v1 .
else
  echo "User mode unkown. Please choose user, root, or leave it out for default user"
fi


