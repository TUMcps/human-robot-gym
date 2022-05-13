docker run -d --name human-robot-gym-gitlab-runner-dind --restart always \
    -v /var/run/docker.sock:/var/run/docker.sock \
    -v /home/thummj/.gitlab-runner:/etc/gitlab-runner \
    gitlab/gitlab-runner:latest

docker run --rm -it -v /home/thummj/.gitlab-runner:/etc/gitlab-runner gitlab/gitlab-runner:latest register