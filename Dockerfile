FROM ghcr.io/educelab/ci-docker:11_v2.dynamic
MAINTAINER Seth Parker <c.seth.parker@uky.edu>

# RUN apt-get update && apt-get upgrade -y 
# RUN apt-get install -y libsdl2-dev

# # Install volcart
# COPY ./ /volume-cartographer-papyrus/
# RUN export CMAKE_PREFIX_PATH="/usr/local/Qt-6.4.2/" \
#     && cmake  \
#       -S /volume-cartographer-papyrus/ \
#       -B /volume-cartographer-papyrus/build/ \
#       -GNinja  \
#       -DCMAKE_BUILD_TYPE=Release  \
#       -DCMAKE_INSTALL_RPATH=/usr/local/Qt-6.4.2/lib \
#       -DSDL2_DIR=/usr/lib/x86_64-linux-gnu/ \
#       -DVC_BUILD_ACVD=ON  \
#     && cmake --build /volume-cartographer-papyrus/build/ \
#     && cmake --install /volume-cartographer-papyrus/build/ \
#     && rm -rf /volume-cartographer-papyrus/

# Start an interactive shell
CMD ["/bin/bash"]
