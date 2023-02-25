#!/bin/bash

if [[ $# -eq 0 ]] ; then
    echo "Usage: $0 VERSION [TAGS]"
    echo "  VERSION      Version tag, Example: 1.23.4-rc1"
    echo "  TAGS         Comma separated list of extra Docker tags"
    exit 1
fi

REPO=ghcr.io/educelab/volume-cartographer
VERSION=${1}
PLATFORMS="linux/amd64,linux/arm64"

IFS=', ' read -r -a TAGS_ARRAY <<< "${2}"
TAGS=""
for tag in "${TAGS_ARRAY[@]}"; do
    TAGS=${TAGS}"-t ${REPO}:${tag} "
done

docker buildx build --platform ${PLATFORMS} --push -t ${REPO}:${VERSION} ${TAGS} -f Dockerfile .
