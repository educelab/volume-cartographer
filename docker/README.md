# Volume Cartographer Docker Image

This directory contains the Dockerfile to build the volcart/volume-cartographer
Docker image. This image contains the
[volume-cartographer](https://gitlab.com/educelab/volume-cartographer)
programs and libraries.

## Requirements
 * Docker 1.10.x or higher

## Building
```shell
# Get a copy of the source
git clone --depth=1 git@gitlab.com:educelab/volume-cartographer.git

# Delete the git info for the clone
rm -rf volume-cartographer/.git

# Build the container
./build.sh
```
