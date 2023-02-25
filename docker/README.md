# Volume Cartographer Docker Image

This directory contains the Dockerfile to build the 
[ghcr.io/educelab/volume-cartographer](https://github.com/educelab/volume-cartographer/pkgs/container/volume-cartographer)
Docker image. 

## Requirements
 * Docker engine 19.03+

## Building
```shell
# Get a copy of the default branch from GitHub
git clone --depth=1 https://github.com/educelab/volume-cartographer.git

# OR copy the current source to the working directory
git clone .. volume-cartographer/

# Build the container locally
docker build -f Dockerfile .

# OR build, tag, and push multi-arch containers with tags (first tag is version)
./build_tag_push.sh 2.24.0

# Build, tag, and push the container with extra tags
./build_tag_push.sh 2.24.0 latest
```
