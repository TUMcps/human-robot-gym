# Run this file either with `./run_docker_train_v2.sh user` or `./run_docker_train_v2.sh root`.
# User mode ensures that the files you create are not made by root.
# Root mode creates a "classic" root user in docker.
# The /runs, /models, and /wandb folders are mounted 
# to store training results outside the docker.

user=${1:-user}
bash_command="/bin/bash"

docker_command="docker run -it"

if [ $2 ]
then
    bash_command="/bin/bash -c ${2}"
    docker_command="docker run"
fi

command="cd human-robot-gym; conda run --no-capture-output -n hrgym pip install -e .; conda run --no-capture-output -n hrgym ${bash_command}"

echo "Chosen mode: $user"
if [ "$user" = "root" ]
then
    ${docker_command} \
    --net=host \
    --volume="$(pwd)/:/root/human-robot-gym/" \
    --shm-size=10.24gb \
    human-robot-gym-train/root:v2 "${command}"
elif [ "$user" = "user" ]
then
    ${docker_command} \
        --net=host \
        --volume="$(pwd)/:/home/$USER/human-robot-gym/" \
        --shm-size=10.24gb \
        human-robot-gym-train/$USER:v2 "${command}"
else
    echo "User mode unknown. Please choose user, root, or leave out for default user"
fi
