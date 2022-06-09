FROM volcart/vcbuilder-debian:latest.dynamic
MAINTAINER Seth Parker <c.seth.parker@uky.edu>

# Install volcart
COPY volume-cartographer/ /volume-cartographer/
RUN mkdir -p /volume-cartographer/build && cd /volume-cartographer/build \
&& export CMAKE_PREFIX_PATH="/usr/local/Qt-6.2.3/" \
&& cmake -GNinja -DVC_BUILD_ACVD=ON -DCMAKE_BUILD_TYPE=Release .. \
&& ninja \
&& ninja install \
&& cd / \
&& rm -rf /volume-cartographer/

# Start an interactive shell
CMD ["/bin/bash"]
