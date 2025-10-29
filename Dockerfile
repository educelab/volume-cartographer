FROM ghcr.io/educelab/ci-docker:dynamic.13.0
LABEL org.opencontainers.image.authors="Seth Parker <c.seth.parker@uky.edu>"

ARG VC_GIT_SHA1

# Install volcart
COPY ./ /volume-cartographer/
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
