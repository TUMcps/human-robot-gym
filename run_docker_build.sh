# Currently, pip install requirements inside the docker only works if you are
# running the docker container as root.
# Consider further tuning of the Docker build process to allow non-root users
# with: --user=1000 \
# --volume="$(pwd):/home/$USER/" \
# --volume="/home/$USER/.mujoco/:/home/$USER/.mujoco/" \
docker run -it \
    --net=host \
    --privileged \
    --volume="$(pwd):/home/human-robot-gym/" \
    --volume="/home/$USER/.mujoco/:/home/.mujoco/" \
    human-robot-gym-build/$USER:v1 \
    bash
