#! /bin/bash
# Run this file either with ./build_docker.sh user or ./build_docker.sh root. 
# User mode will create a user in docker so that the files you create are not made by root.
# Root mode creates a "classic" root user in docker.
# Currently, only root allows building safety_shield_py properly. Sorry.

user=${1:-root}
echo "Chosen user mode"=$user
if [ "$user" = "root" ]
then
    docker run -it \
        --volume="$(pwd):/home/human-robot-gym/" \
        --volume="/home/$USER/.mujoco/:/home/.mujoco/" \
        --net=host \
        --privileged \
        human-robot-gym-train/$USER:v1
elif [ "$user" = "user" ]
then
    docker run -it \
        --user "$(id -u):$(id -g)" \
        --volume="$(pwd):/home/$USER/human-robot-gym/" \
        --volume="/home/$USER/.mujoco/:/home/$USER/.mujoco/" \
        --net=host \
        --privileged \
        --entrypoint bash \
        human-robot-gym-train/$USER:v1
else
    echo "User mode unkown. Please choose user, root, or leave it out for default user"
fi




