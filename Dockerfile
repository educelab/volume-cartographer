FROM ghcr.io/educelab/ci-docker:dynamic.12.1.1-dev3
LABEL org.opencontainers.image.authors="Seth Parker <c.seth.parker@uky.edu>"

ARG VC_GIT_SHA1

#ENV LANG=en_US.UTF-8
#ENV LANGUAGE=en_US:en
#ENV LC_ALL=en_US.UTF-8
ENV QT_PLUGIN_PATH=/usr/local/Qt-6.7.2/plugins

# Install volcart
COPY ./ /volume-cartographer/
RUN export CMAKE_PREFIX_PATH="/usr/local/Qt-6.7.2/" \
    && cmake  \
      -S /volume-cartographer/ \
      -B /volume-cartographer/build/ \
      -GNinja  \
      -DCMAKE_BUILD_TYPE=Release  \
      -DCMAKE_INSTALL_RPATH=/usr/local/Qt-6.7.2/lib \
      -DVC_BUILD_ACVD=ON  \
    && cmake --build /volume-cartographer/build/ \
    && cmake --install /volume-cartographer/build/ \
    && rm -rf /volume-cartographer/

# Start an interactive shell
ENTRYPOINT ["/bin/bash", "-c"]
CMD ["vc_version"]
