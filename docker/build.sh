#!/bin/bash

REPO=volcart/volume-cartographer
VERSION=latest

docker build -t ${REPO}:${VERSION} -f Dockerfile .
