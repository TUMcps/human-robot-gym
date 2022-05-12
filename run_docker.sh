#! /bin/bash
# Run this file either with ./build_docker.sh user or ./build_docker.sh root. 
# User mode will create a user in docker so that the files you create are not made by root.
# Root mode creates a "classic" root user in docker.
# Currently, only root allows building safety_shield_py properly. Sorry.
# You can add the argument gui to allow graphical output.
# 
# If not working, first do: sudo rm -rf /tmp/.docker.xauth
# It still not working, try running the script as root.
user=${1:-root}
echo "Chosen user mode"=$user
gui=${2:-none}
echo "Chosen user mode"=$gui

if [ "$gui" = "gui" ]
then
    XAUTH=/tmp/.docker.xauth

    echo "Preparing Xauthority data..."
    xauth_list=$(xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/')
    if [ ! -f $XAUTH ]; then
        if [ ! -z "$xauth_list" ]; then
            echo $xauth_list | xauth -f $XAUTH nmerge -
        else
            touch $XAUTH
        fi
        chmod a+r $XAUTH
    fi

    echo "Done."
    echo ""
    echo "Verifying file contents:"
    file $XAUTH
    echo "--> It should say \"X11 Xauthority data\"."
    echo ""
    echo "Permissions:"
    ls -FAlh $XAUTH
    echo ""
    echo "Running docker..."

    if [ "$user" = "root" ]
    then
    docker run -it \
        --gpus=all \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --env="XAUTHORITY=$XAUTH" \
        --volume="$XAUTH:$XAUTH" \
        --volume="$(pwd):/home/human-robot-gym/" \
        --volume="/home/$USER/.mujoco/:/home/.mujoco/" \
        --net=host \
        --privileged \
        --runtime=nvidia \
        human-robot-gym/$USER:v1
    elif [ "$user" = "user" ]
    then
    docker run -it \
        --user "$(id -u):$(id -g)" \
        --gpus=all \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --env="XAUTHORITY=$XAUTH" \
        --volume="$XAUTH:$XAUTH" \
        --volume="$(pwd):/home/$USER/human-robot-gym/" \
        --volume="/home/$USER/.mujoco/:/home/$USER/.mujoco/" \
        --net=host \
        --privileged \
        --runtime=nvidia \
        --entrypoint bash \
        human-robot-gym/$USER:v1
    else
    echo "User mode unkown. Please choose user, root, or leave it out for default user"
    fi
else
    if [ "$user" = "root" ]
    then
        docker run -it \
            --gpus=all \
            --volume="$(pwd):/home/human-robot-gym/" \
            --volume="/home/$USER/.mujoco/:/home/.mujoco/" \
            --net=host \
            --privileged \
            --runtime=nvidia \
            human-robot-gym/$USER:v1
    elif [ "$user" = "user" ]
    then
        docker run -it \
            --user "$(id -u):$(id -g)" \
            --gpus=all \
            --volume="$(pwd):/home/$USER/human-robot-gym/" \
            --volume="/home/$USER/.mujoco/:/home/$USER/.mujoco/" \
            --net=host \
            --privileged \
            --runtime=nvidia \
            --entrypoint bash \
            human-robot-gym/$USER:v1
    else
        echo "User mode unkown. Please choose user, root, or leave it out for default user"
    fi
fi



