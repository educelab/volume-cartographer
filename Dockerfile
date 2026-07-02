FROM ghcr.io/educelab/ci-docker:dynamic.12.2
LABEL org.opencontainers.image.authors="Seth Parker <c.seth.parker@uky.edu>"

ARG VC_GIT_SHA1

COPY ./ /volume-cartographer/

# Not packaged for apt or bundled in this base image; must be installed for find_package().
RUN apt-get update \
    && apt-get install -y --no-install-recommends nlohmann-json3-dev \
    && /volume-cartographer/scripts/ci-install-smgl-libcore.sh \
    && rm -rf /var/lib/apt/lists/* \
    && ldconfig

# Install volcart
RUN export CMAKE_PREFIX_PATH="/usr/local/Qt-6.10.0/" \
    && cmake  \
      -S /volume-cartographer/ \
      -B /volume-cartographer/build/ \
      -GNinja  \
      -DCMAKE_BUILD_TYPE=Release  \
      -DCMAKE_INSTALL_RPATH=/usr/local/Qt-6.10.0/lib \
      -DVC_BUILD_ACVD=ON  \
    && cmake --build /volume-cartographer/build/ \
    && cmake --install /volume-cartographer/build/ \
    && rm -rf /volume-cartographer/

# Report the version
CMD ["vc_version"]
