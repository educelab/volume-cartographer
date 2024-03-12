FROM ghcr.io/educelab/ci-docker:dynamic.12.0
MAINTAINER Seth Parker <c.seth.parker@uky.edu>

# Install volcart
COPY ./ /volume-cartographer/
RUN export CMAKE_PREFIX_PATH="/usr/local/Qt-6.6.1/" \
    && cmake  \
      -S /volume-cartographer/ \
      -B /volume-cartographer/build/ \
      -GNinja  \
      -DCMAKE_BUILD_TYPE=Release  \
      -DCMAKE_INSTALL_RPATH=/usr/local/Qt-6.6.1/lib \
      -DVC_BUILD_ACVD=ON  \
    && cmake --build /volume-cartographer/build/ \
    && cmake --install /volume-cartographer/build/ \
    && rm -rf /volume-cartographer/

# Start an interactive shell
CMD ["/bin/bash"]
