FROM ghcr.io/educelab/ci-docker:dynamic.12.2
LABEL org.opencontainers.image.authors="Seth Parker <c.seth.parker@uky.edu>"

ARG VC_GIT_SHA1

COPY ./ /volume-cartographer/

# smgl and libcore aren't packaged for apt or bundled in this base image, so
# build them from source for find_package(). VC is built static below, so static deps.
RUN apt-get update \
    && apt-get install -y --no-install-recommends curl nlohmann-json3-dev \
    && mkdir -p /tmp/smgl \
    && curl -fsSL https://github.com/educelab/smgl/archive/refs/tags/v0.11.0-rc.2.tar.gz | tar -xz --strip-components=1 -C /tmp/smgl \
    && cmake -S /tmp/smgl -B /tmp/smgl/build -GNinja -DCMAKE_BUILD_TYPE=Release \
        -DSMGL_BUILD_JSON=OFF -DSMGL_USE_BOOSTFS=OFF -DSMGL_BUILD_TESTS=OFF -DSMGL_BUILD_DOCS=OFF \
    && cmake --build /tmp/smgl/build && cmake --install /tmp/smgl/build \
    && mkdir -p /tmp/libcore \
    && curl -fsSL https://github.com/educelab/libcore/archive/refs/tags/v0.3.0-rc.1.tar.gz | tar -xz --strip-components=1 -C /tmp/libcore \
    && cmake -S /tmp/libcore -B /tmp/libcore/build -GNinja -DCMAKE_BUILD_TYPE=Release \
        -DEDUCE_CORE_BUILD_TESTS=OFF -DEDUCE_CORE_BUILD_DOCS=OFF -DEDUCE_CORE_BUILD_EXAMPLES=OFF \
    && cmake --build /tmp/libcore/build && cmake --install /tmp/libcore/build \
    && rm -rf /tmp/smgl /tmp/libcore /var/lib/apt/lists/* \
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
