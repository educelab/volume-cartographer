#pragma once

#include <cstdint>

namespace volcart::protocol
{

/**
 * Magic value that identifies this protocol on the wire. Generated with:
 * `echo -n Volume Protocol | md5sum | cut -c 1-8`
 */
constexpr uint32_t MAGIC = 0xf6fcdac0;

/** Size of a volpkg identifier. */
constexpr uint32_t VOLPKG_SZ = 64;

/** Size of a volume identifier. */
constexpr uint32_t VOLUME_SZ = 64;

/** Enumeration of protocol versions. */
enum Version : uint8_t { V1 = 1 };

// TODO: Add a request/response flag so that we can share a uniform prefix
// header for all packets.

/** Packet header for requests. */
struct RequestHdr {
    uint32_t magic{MAGIC};
    Version version{Version::V1};
    uint8_t pad[3];
    uint32_t numRequests{0};
};

/** Packet structure for arguments to a given request. */
struct RequestArgs {
    // TODO: Move volpkg and volume to UUIDs (128-bit) once the VolumePkg class
    // is converted to use UUIDs (v4).
    char volpkg[VOLPKG_SZ];
    char volume[VOLUME_SZ];
    float centerX;
    float centerY;
    float centerZ;
    float basis0X;
    float basis0Y;
    float basis0Z;
    float basis1X;
    float basis1Y;
    float basis1Z;
    float basis2X;
    float basis2Y;
    float basis2Z;
    float samplingRX;
    float samplingRY;
    float samplingRZ;
    float samplingInterval;
};

/** Packet structure for a response to a request. */
struct ResponseArgs {
    char volpkg[VOLPKG_SZ];
    char volume[VOLUME_SZ];
    uint32_t extentX;
    uint32_t extentY;
    uint32_t extentZ;
    uint32_t size;
};

}  // namespace volcart::protocol
