#include "vc/core/io/TIFFIO.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>

#include <opencv2/imgproc.hpp>

#include "vc/core/Version.hpp"
#include "vc/core/io/FileFilters.hpp"
#include "vc/core/types/Exceptions.hpp"
#include "vc/core/util/Logging.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

// Wrapping in a namespace to avoid define collisions
namespace lt
{
#include <tiffio.h>
}

namespace vc = volcart;
namespace tio = volcart::tiffio;
namespace fs = volcart::filesystem;

namespace
{
// Return a CV Mat type using TIFF type (signed, unsigned, float),
// bit-depth, and number of channels
auto GetCVMatType(
    const std::uint16_t tifType,
    const std::uint16_t depth,
    const std::uint16_t channels) -> int
{
    switch (depth) {
        case 8:
            if (tifType == SAMPLEFORMAT_INT) {
                return CV_MAKETYPE(CV_8S, channels);
            } else {
                return CV_MAKETYPE(CV_8U, channels);
            }
        case 16:
            if (tifType == SAMPLEFORMAT_INT) {
                return CV_MAKETYPE(CV_16S, channels);
            } else {
                return CV_MAKETYPE(CV_16U, channels);
            }
        case 32:
            if (tifType == SAMPLEFORMAT_INT) {
                return CV_MAKETYPE(CV_32S, channels);
            } else {
                return CV_MAKETYPE(CV_32F, channels);
            }
        default:
            return CV_8UC3;
    }
}

constexpr std::size_t MAX_TIFF_BYTES{4'294'967'296};
constexpr std::size_t BITS_PER_BYTE{8};

inline auto NeedBigTIFF(
    std::size_t w, std::size_t h, std::size_t cns, std::size_t bps) -> bool
{
    const std::size_t bytes = w * h * cns * bps / BITS_PER_BYTE;
    return bytes >= MAX_TIFF_BYTES;
}

}  // namespace

auto tio::ReadTIFF(const volcart::filesystem::path& path) -> cv::Mat
{
    // Make sure input file exists
    if (not fs::exists(path)) {
        throw IOException("File does not exist");
    }

    // Open the file read-only
    lt::TIFF* tif = lt::TIFFOpen(path.c_str(), "rc");
    if (tif == nullptr) {
        throw IOException("Failed to open TIFF");
    }

    // Get metadata
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t rowsPerStrip = 0;
    std::uint16_t type = 1;
    std::uint16_t depth = 1;
    std::uint16_t channels = 1;
    std::uint16_t config = 0;
    Compression compression = Compression::NONE;
    TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &width);
    TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &height);
    TIFFGetField(tif, TIFFTAG_SAMPLEFORMAT, &type);
    TIFFGetField(tif, TIFFTAG_BITSPERSAMPLE, &depth);
    TIFFGetField(tif, TIFFTAG_SAMPLESPERPIXEL, &channels);
    TIFFGetField(tif, TIFFTAG_PLANARCONFIG, &config);
    TIFFGetField(tif, TIFFTAG_COMPRESSION, &compression);
    TIFFGetField(tif, TIFFTAG_ROWSPERSTRIP, &rowsPerStrip);
    auto cvType = ::GetCVMatType(type, depth, channels);

    // We limit ourselved probably a bit more than necessary,
    // but better safe than sorry
    auto canMMap =
        config == PLANARCONFIG_CONTIG and type == SAMPLEFORMAT_UINT and
        depth == 16 and channels == 1 and compression == Compression::NONE and
        rowsPerStrip == height;  // important, full image is in a single strip

    // Construct the mat
    auto h = static_cast<int>(height);
    auto w = static_cast<int>(width);
    cv::Mat img;

    if (canMMap) {
        // Assumes there's only one, i.e. rows == height
        std::uint32_t* stripOffset = 0;
        int res = TIFFGetField(tif, TIFFTAG_STRIPOFFSETS, &stripOffset);

        // Open and mmap TIFF file
        int fd = open(path.c_str(), O_RDONLY);
        if (fd == -1) {
            throw IOException("Failed to open TIFF: " + path.string());
        }
        struct stat sb;
        if (fstat(fd, &sb) == -1) {
            throw IOException("Failed to fstat TIFF: " + path.string());
        }

        void* data = mmap(nullptr, sb.st_size, PROT_READ, MAP_SHARED, fd, 0);
        if (data == MAP_FAILED) {
            // Print error code
            printf("mmap() errno: %d\n", errno);
            throw IOException("Failed to mmap TIFF: " + path.string());
        }
        close(fd);

        img = cv::Mat(h, w, cvType, (char*)data + stripOffset[0]);
    } else {
        // Load the old way via TIFF library
        vc::Logger()->debug(
            "Cannot mmap TIFF (width: %d height: %d config: %d type: %d depth: "
            "%d channel: %d rowsPerStrip: %d compression: %s) => loading the "
            "old way",
            width, height, config, type, depth, channels, rowsPerStrip,
            (compression != Compression::NONE ? "true" : "false"));

        img = cv::Mat::zeros(h, w, cvType);

        // Read the rows
        auto bufferSize = static_cast<size_t>(lt::TIFFScanlineSize(tif));
        std::vector<char> buffer(bufferSize + 4);
        if (config == PLANARCONFIG_CONTIG) {
            for (auto row = 0; row < height; row++) {
                lt::TIFFReadScanline(tif, &buffer[0], row);
                std::memcpy(img.ptr(row), &buffer[0], bufferSize);
            }
        } else if (config == PLANARCONFIG_SEPARATE) {
            throw IOException(
                "Unsupported TIFF planar configuration: PLANARCONFIG_SEPARATE");
        }

        // Do channel conversion
        auto cvtNeeded = img.channels() == 3 or img.channels() == 4;
        auto cvtSupported = img.depth() != CV_8S and img.depth() != CV_16S and
                            img.depth() != CV_32S;
        if (cvtNeeded) {
            if (cvtSupported) {
                if (img.channels() == 3) {
                    cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
                } else if (img.channels() == 4) {
                    cv::cvtColor(img, img, cv::COLOR_RGBA2BGRA);
                }
            } else {
                vc::Logger()->warn(
                    "[TIFFIO] RGB->BGR conversion for signed 8-bit and 16-bit "
                    "images is not supported. Image will be loaded with RGB "
                    "element order.");
            }
        }
    }

    lt::TIFFClose(tif);

    return img;
}

// Write a TIFF to a file. This implementation heavily borrows from how OpenCV's
// TIFFEncoder writes to the TIFF
void tio::WriteTIFF(
    const fs::path& path, const cv::Mat& img, Compression compression)
{
    // Safety checks
    if (img.channels() < 1 or img.channels() > 4) {
        throw IOException("Unsupported number of channels");
    }

    if (not io::FileExtensionFilter(path, {"tif", "tiff"})) {
        throw IOException(
            "Invalid file extension " + path.extension().string());
    }

    // Image metadata
    auto channels = img.channels();
    auto width = static_cast<unsigned>(img.cols);
    auto height = static_cast<unsigned>(img.rows);
    auto rowsPerStrip = height;

    // Sample format
    int bitsPerSample;
    int sampleFormat;
    switch (img.depth()) {
        case CV_8U:
            sampleFormat = SAMPLEFORMAT_UINT;
            bitsPerSample = 8;
            break;
        case CV_8S:
            sampleFormat = SAMPLEFORMAT_INT;
            bitsPerSample = 8;
            break;
        case CV_16U:
            sampleFormat = SAMPLEFORMAT_UINT;
            bitsPerSample = 16;
            break;
        case CV_16S:
            sampleFormat = SAMPLEFORMAT_INT;
            bitsPerSample = 16;
            break;
        case CV_32S:
            sampleFormat = SAMPLEFORMAT_INT;
            bitsPerSample = 32;
            break;
        case CV_32F:
            sampleFormat = SAMPLEFORMAT_IEEEFP;
            bitsPerSample = 32;
            break;
        case CV_64F:
            sampleFormat = SAMPLEFORMAT_IEEEFP;
            bitsPerSample = 64;
            break;
        default:
            throw IOException("Unsupported image depth");
    }

    // Photometric Interpretation
    int photometric;
    switch (channels) {
        case 1:
        case 2:
            photometric = PHOTOMETRIC_MINISBLACK;
            break;
        case 3:
        case 4:
            photometric = PHOTOMETRIC_RGB;
            break;
        default:
            throw IOException("Unsupported number of channels");
    }

    // Get working copy with converted channels if an RGB-type image
    auto cvtNeeded = img.channels() == 3 or img.channels() == 4;
    auto cvtSupported = img.depth() != CV_8S and img.depth() != CV_16S and
                        img.depth() != CV_32S;
    cv::Mat imgCopy;
    if (cvtNeeded and cvtSupported) {
        if (img.channels() == 3) {
            cv::cvtColor(img, imgCopy, cv::COLOR_BGR2RGB);
        } else if (img.channels() == 4) {
            cv::cvtColor(img, imgCopy, cv::COLOR_BGRA2RGBA);
        }
    } else if (cvtNeeded) {
        throw IOException(
            "BGR->RGB conversion for signed 8-bit and 16-bit images is not "
            "supported.");
    } else {
        imgCopy = img;
    }

    // Estimated file size in bytes
    auto useBigTIFF = ::NeedBigTIFF(width, height, channels, bitsPerSample);
    if (useBigTIFF) {
        Logger()->warn("File estimate >= 4GB. Writing as BigTIFF.");
    }

    // Open the file
    const std::string mode = (useBigTIFF) ? "w8" : "w";
    auto* out = lt::TIFFOpen(path.c_str(), mode.c_str());
    if (out == nullptr) {
        Logger()->error("Failed to open file for writing: {}", path.string());
        throw IOException("Failed to open file for writing: " + path.string());
    }

    // Encoding parameters
    lt::TIFFSetField(out, TIFFTAG_IMAGEWIDTH, width);
    lt::TIFFSetField(out, TIFFTAG_IMAGELENGTH, height);
    lt::TIFFSetField(out, TIFFTAG_PHOTOMETRIC, photometric);
    lt::TIFFSetField(out, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
    lt::TIFFSetField(out, TIFFTAG_COMPRESSION, compression);
    lt::TIFFSetField(out, TIFFTAG_SAMPLEFORMAT, sampleFormat);
    lt::TIFFSetField(out, TIFFTAG_BITSPERSAMPLE, bitsPerSample);
    lt::TIFFSetField(out, TIFFTAG_SAMPLESPERPIXEL, channels);
    lt::TIFFSetField(out, TIFFTAG_ROWSPERSTRIP, rowsPerStrip);

    // Add alpha tag data
    // TODO: Let user decide associated/unassociated tag
    // See TIFF 6.0 spec, section 18
    if (channels == 2 or channels == 4) {
        std::array<std::uint16_t, 1> tag{EXTRASAMPLE_UNASSALPHA};
        lt::TIFFSetField(out, TIFFTAG_EXTRASAMPLES, 1, tag.data());
    }

    // Metadata
    lt::TIFFSetField(
        out, TIFFTAG_SOFTWARE, ProjectInfo::NameAndVersion().c_str());

    // Row buffer. OpenCV documentation mentions that TIFFWriteScanline
    // modifies its read buffer, so we can't use the cv::Mat directly
    auto bufferSize = static_cast<std::size_t>(lt::TIFFScanlineSize(out));
    std::vector<char> buffer(bufferSize + 32);

    // For each row
    for (unsigned row = 0; row < height; row++) {
        std::memcpy(&buffer[0], imgCopy.ptr(row), bufferSize);
        auto result = lt::TIFFWriteScanline(out, &buffer[0], row, 0);
        if (result == -1) {
            lt::TIFFClose(out);
            auto msg = "Failed to write row " + std::to_string(row);
            throw IOException(msg);
        }
    }

    // Close the TIFF
    lt::TIFFClose(out);
}
