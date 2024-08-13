#include "vc/core/io/TIFFIO.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>

#include <opencv2/imgproc.hpp>

#include "vc/core/Version.hpp"
#include "vc/core/io/FileFilters.hpp"
#include "vc/core/types/Exceptions.hpp"
#include "vc/core/util/Logging.hpp"

#ifdef _MSC_VER
// TODO: Implement memmap for Windows
#else
// For memmaping
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

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
        case 8: {
            if (tifType == SAMPLEFORMAT_INT) {
                return CV_MAKETYPE(CV_8S, channels);
            }
            return CV_MAKETYPE(CV_8U, channels);
        }
        case 16: {
            if (tifType == SAMPLEFORMAT_INT) {
                return CV_MAKETYPE(CV_16S, channels);
            }
            return CV_MAKETYPE(CV_16U, channels);
        }
        case 32: {
            if (tifType == SAMPLEFORMAT_INT) {
                return CV_MAKETYPE(CV_32S, channels);
            }
            return CV_MAKETYPE(CV_32F, channels);
        }
        default:
            return CV_8UC3;
    }
}

constexpr std::size_t MAX_TIFF_BYTES{4'294'967'296};
constexpr std::size_t BITS_PER_BYTE{8};

auto NeedBigTIFF(
    const std::size_t w,
    const std::size_t h,
    const std::size_t cns,
    const std::size_t bps) -> bool
{
    const std::size_t bytes = w * h * cns * bps / BITS_PER_BYTE;
    return bytes >= MAX_TIFF_BYTES;
}

struct TIFFHeader {
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t rowsPerStrip = 0;
    std::uint16_t type = 1;
    std::uint16_t depth = 1;
    std::uint16_t channels = 1;
    std::uint16_t config = 0;
    std::vector<std::uint64_t> stripOffsets;
    tio::Compression compression{tio::Compression::NONE};
};

auto ReadHeader(lt::TIFF* tif)
{
    // Get metadata
    TIFFHeader hdr;
    TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &hdr.width);
    TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &hdr.height);
    TIFFGetField(tif, TIFFTAG_SAMPLEFORMAT, &hdr.type);
    TIFFGetField(tif, TIFFTAG_BITSPERSAMPLE, &hdr.depth);
    TIFFGetField(tif, TIFFTAG_SAMPLESPERPIXEL, &hdr.channels);
    TIFFGetField(tif, TIFFTAG_PLANARCONFIG, &hdr.config);
    TIFFGetField(tif, TIFFTAG_COMPRESSION, &hdr.compression);
    TIFFGetField(tif, TIFFTAG_ROWSPERSTRIP, &hdr.rowsPerStrip);
    // TODO: This is wrong and isn't loading the correct offsets
    hdr.stripOffsets =
        std::vector<std::uint64_t>(hdr.height / hdr.rowsPerStrip, 0);
    TIFFGetField(tif, TIFFTAG_STRIPOFFSETS, hdr.stripOffsets.data());
    return hdr;
}

auto ReadImage(lt::TIFF* tif, const TIFFHeader& hdr) -> cv::Mat
{
    // Construct the mat
    const auto h = static_cast<int>(hdr.height);
    const auto w = static_cast<int>(hdr.width);
    const auto cvType = GetCVMatType(hdr.type, hdr.depth, hdr.channels);
    cv::Mat img = cv::Mat::zeros(h, w, cvType);

    // Read the rows
    const auto bufferSize = static_cast<size_t>(lt::TIFFScanlineSize(tif));
    std::vector<char> buffer(bufferSize + 4);
    if (hdr.config == PLANARCONFIG_CONTIG) {
        for (auto row = 0; row < hdr.height; row++) {
            lt::TIFFReadScanline(tif, &buffer[0], row);
            std::memcpy(img.ptr(row), &buffer[0], bufferSize);
        }
    } else if (hdr.config == PLANARCONFIG_SEPARATE) {
        throw volcart::IOException(
            "Unsupported TIFF planar configuration: PLANARCONFIG_SEPARATE");
    }

    // Do channel conversion
    const auto cvtNeeded = img.channels() == 3 or img.channels() == 4;
    const auto cvtSupported = img.depth() != CV_8S and img.depth() != CV_16S and
                              img.depth() != CV_32S;
    if (cvtNeeded) {
        if (cvtSupported) {
            if (img.channels() == 3) {
                cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
            } else if (img.channels() == 4) {
                cv::cvtColor(img, img, cv::COLOR_RGBA2BGRA);
            }
        } else {
            volcart::Logger()->warn(
                "[TIFFIO] RGB->BGR conversion for signed 8-bit and 16-bit "
                "images is not supported. Image will be loaded with RGB "
                "element order.");
        }
    }

    return img;
}

#ifdef _MSC_VER
// TODO: Implement memmap for Windows
#else
auto CanMMap(const TIFFHeader& hdr) -> bool
{
    auto res = hdr.config == PLANARCONFIG_CONTIG;
    res &= hdr.type == SAMPLEFORMAT_UINT;
    res &= hdr.depth == 16 and hdr.channels == 1;
    res &= hdr.compression == tio::Compression::NONE;
    // important: full image is in a single strip
    res &= hdr.rowsPerStrip == hdr.height;
    return res;
}

// Memory maps the TIF image found at path
// File must pass CanMMap first
auto MMapImage(const fs::path& path, const TIFFHeader& hdr)
    -> std::pair<cv::Mat, tio::mmap_info>
{
    vc::Logger()->trace("[TIFFIO] Memory mapping file: {}", path.string());
    // Open and mmap TIFF file
    const int fd = open(path.c_str(), O_RDONLY);
    if (fd == -1) {
        throw vc::IOException("Failed to open TIFF: " + path.string());
    }

    struct stat sb {
    };
    if (fstat(fd, &sb) == -1) {
        throw vc::IOException("Failed to fstat TIFF: " + path.string());
    }

    auto* data = mmap(nullptr, sb.st_size, PROT_READ, MAP_SHARED, fd, 0);
    if (data == MAP_FAILED) {
        vc::Logger()->error("[TIFFIO] mmap() errno: {}", errno);
        throw vc::IOException("Failed to mmap TIFF: " + path.string());
    }
    close(fd);

    const auto h = static_cast<int>(hdr.height);
    const auto w = static_cast<int>(hdr.width);
    const auto cvType = GetCVMatType(hdr.type, hdr.depth, hdr.channels);
    cv::Mat img(h, w, cvType, static_cast<char*>(data) + hdr.stripOffsets[0]);
    return {img, {.addr = data, .size = sb.st_size}};
}
#endif

}  // namespace

auto tio::ReadTIFF(const fs::path& path, mmap_info* mmap_info) -> cv::Mat
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
    const auto hdr = ReadHeader(tif);
    cv::Mat img;

#ifdef _MSC_VER
    // Read into the mat
    if (memmap) {
        Logger()->debug("[TIFFIO] Memmap is not supported on this platform");
    }
    img = ReadImage(tif, hdr);
#else
    // Load memmap'd image
    const auto canMMap = CanMMap(hdr);
    if (mmap_info and canMMap) {
        std::tie(img, *mmap_info) = MMapImage(path, hdr);
    } else {
        if (not canMMap) {
            Logger()->debug(
                "[TIFFIO] Cannot memory map TIFF: {}. Image will be "
                "read into memory instead.",
                path.string());
        }
        img = ReadImage(tif, hdr);
    }
#endif

    // Close the tif file
    lt::TIFFClose(tif);
    return img;
}

auto tio::UnmapTIFF(const mmap_info& mmap_info)
{
#ifdef _MSC_VER
    return;
#else
    if (not mmap_info.addr) {
        Logger()->debug("Empty address");
        return;
    }
    if (mmap_info.size < 1) {
        Logger()->debug("Invalid mapping size: {}", mmap_info.size);
        return;
    }

    munmap(mmap_info.addr, mmap_info.size);
#endif
}

// Write a TIFF to a file. This implementation heavily borrows from how OpenCV's
// TIFFEncoder writes to the TIFF
void tio::WriteTIFF(
    const fs::path& path, const cv::Mat& img, const Compression compression)
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
