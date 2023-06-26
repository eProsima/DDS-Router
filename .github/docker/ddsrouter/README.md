# DDS ROUTER TEST DOCKER

In order to build this docker image, use command in current directory:

```sh
docker build --rm -t ddsrouter_test:some_tag --build-arg "fastdds_branch=master" --build-arg "devutils_branch=main" --build-arg "ddspipe_branch=main" --build-arg "ddsrouter_branch=main" .
```
