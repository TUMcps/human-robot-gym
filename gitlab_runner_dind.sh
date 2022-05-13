docker run -d --name human-robot-gym-gitlab-runner-dind --restart always \
  -v /srv/gitlab-runner/config:/etc/gitlab-runner \
  -v /var/run/docker.sock:/var/run/docker.sock \
  gitlab/gitlab-runner:latest

docker run --rm -it -v /srv/gitlab-runner/config:/etc/gitlab-runner gitlab/gitlab-runner register -n \
  --url https://gitlab.lrz.de \
  --registration-token $GITLABTOKEN \
  --executor docker \
  --description "DinD runner" \
  --docker-image "docker:19.03.12" \
  --docker-privileged \
  --docker-volumes "/certs/client"
