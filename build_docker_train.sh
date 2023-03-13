# Run this file either with `./build_docker_train_v2.sh user` or `./build_docker_train_v2.sh root`.
# User mode will create a user in the docker so that the files you create are not made by root.
# Root mode creates a "classic" root user in docker.

user=${1:-user}
echo "Chosen mode: $user"

if [ "$user" = "root" ]
then
    DOCKER_BUILDKIT=1 docker build \
        -f Dockerfile.train \
        --build-arg MODE=root \
        -t human-robot-gym-train/root:v2 .
elif [ "$user" = "user" ]
then
    DOCKER_BUILDKIT=1 docker build \
        -f Dockerfile.train \
        --build-arg MODE=user \
        --build-arg USER_UID=$(id -u) \
        --build-arg USER_GID=$(id -g) \
        --build-arg USERNAME=$USER \
        -t human-robot-gym-train/$USER:v2 .
else
  echo "User mode unkown. Please choose user, root, or leave it out for default user"
fi
