# Run this file either with `./run_docker_train_v2.sh user` or `./run_docker_train_v2.sh root`.
# User mode ensures that the files you create are not made by root.
# Root mode creates a "classic" root user in docker.
# The /runs, /models, and /wandb folders are mounted 
# to store training results outside the docker.

user=${1:-user}
echo "Chosen mode: $user"
if [ "$user" = "root" ]
then
    docker run -it \
    --net=host \
    --volume="$(pwd)/:/root/human-robot-gym/" \
    human-robot-gym-train/root:v2
elif [ "$user" = "user" ]
then
    docker run -it \
        --net=host \
        --volume="$(pwd)/:/home/$USER/human-robot-gym/" \
        human-robot-gym-train/$USER:v2
else
    echo "User mode unknown. Please choose user, root, or leave out for default user"
fi
