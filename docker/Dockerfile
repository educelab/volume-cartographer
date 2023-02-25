FROM ghcr.io/educelab/ci-docker:11_v2.dynamic
MAINTAINER Seth Parker <c.seth.parker@uky.edu>
LABEL org.opencontainers.image.description "Volume Cartographer libraries and tools"
LABEL org.opencontainers.image.source https://github.com/educelab/volume-cartographer/
LABEL org.opencontainers.image.licenses GPL-3.0

# Install volcart
COPY volume-cartographer/ /volume-cartographer/
RUN mkdir -p /volume-cartographer/build && cd /volume-cartographer/build && \
    export CMAKE_PREFIX_PATH="/usr/local/Qt-6.4.2/" && \
    cmake  \
      -GNinja  \
      -DVC_BUILD_ACVD=ON  \
      -DCMAKE_BUILD_TYPE=Release  \
      -DVC_BUILD_EXAMPLES=OFF  \
      -DVC_BUILD_TESTS=OFF  \
      -DCMAKE_INSTALL_RPATH=/usr/local/Qt-6.4.2/lib \
      .. && \
    ninja && \
    ninja install &&  \
    cd / && \
    rm -rf /volume-cartographer/

# Start an interactive shell
CMD ["/bin/bash"]
