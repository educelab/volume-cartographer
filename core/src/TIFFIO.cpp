#include "vc/core/io/TIFFIO.hpp"

#include <cstring>

#include <opencv2/imgproc.hpp>

#include "vc/core/Version.hpp"
#include "vc/core/io/FileExtensionFilter.hpp"

// Wrapping in a namespace to avoid define collisions
namespace lt
{
#include <tiffio.h>
}

namespace tio = volcart::tiffio;
namespace fs = boost::filesystem;

// Write a TIFF to a file. This implementation heavily borrows from how OpenCV's
// TIFFEncoder writes to the TIFF
void tio::WriteTIFF(const fs::path& path, const cv::Mat& img)
{
    // Safety checks
    if (img.channels() < 1 or img.channels() > 4) {
        throw std::runtime_error("Unsupported number of channels");
    }

    if (not io::FileExtensionFilter(path, {"tif", "tiff"})) {
        throw std::runtime_error(
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
            throw std::runtime_error("Unsupported image depth");
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
            throw std::runtime_error("Unsupported number of channels");
    }

    // Open the file
    auto out = lt::TIFFOpen(path.c_str(), "w");
    if (out == nullptr) {
        throw std::runtime_error("Failed to open file for writing");
    }

    // Encoding parameters
    lt::TIFFSetField(out, TIFFTAG_IMAGEWIDTH, width);
    lt::TIFFSetField(out, TIFFTAG_IMAGELENGTH, height);
    lt::TIFFSetField(out, TIFFTAG_PHOTOMETRIC, photometric);
    lt::TIFFSetField(out, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
    lt::TIFFSetField(out, TIFFTAG_COMPRESSION, COMPRESSION_LZW);
    lt::TIFFSetField(out, TIFFTAG_SAMPLEFORMAT, sampleFormat);
    lt::TIFFSetField(out, TIFFTAG_BITSPERSAMPLE, bitsPerSample);
    lt::TIFFSetField(out, TIFFTAG_SAMPLESPERPIXEL, channels);
    lt::TIFFSetField(out, TIFFTAG_ROWSPERSTRIP, rowsPerStrip);

    // Add alpha tag data
    // TODO: Let user decide associated/unassociated tag
    // See TIFF 6.0 spec, section 18
    if (channels == 2 or channels == 4) {
        std::array<uint16_t, 1> tag{EXTRASAMPLE_UNASSALPHA};
        lt::TIFFSetField(out, TIFFTAG_EXTRASAMPLES, 1, tag.data());
    }

    // Metadata
    lt::TIFFSetField(
        out, TIFFTAG_SOFTWARE, ProjectInfo::NameAndVersion().c_str());

    // Row buffer. OpenCV documentation mentions that TIFFWriteScanline
    // modifies its read buffer, so we can't use the cv::Mat directly
    auto bufferSize = static_cast<size_t>(lt::TIFFScanlineSize(out));
    std::vector<char> buffer(bufferSize + 32);

    // Get working copy with converted channels if an RGB-type image
    cv::Mat imgCopy;
    if (img.channels() == 3) {
        cv::cvtColor(img, imgCopy, cv::COLOR_BGR2RGB);
    } else if (img.channels() == 4) {
        cv::cvtColor(img, imgCopy, cv::COLOR_BGRA2RGBA);
    } else {
        imgCopy = img;
    }

    // For each row
    for (unsigned row = 0; row < height; row++) {
        std::memcpy(&buffer[0], imgCopy.ptr(row), bufferSize);
        auto result = lt::TIFFWriteScanline(out, &buffer[0], row, 0);
        if (result == -1) {
            lt::TIFFClose(out);
            auto msg = "Failed to write row " + std::to_string(row);
            throw std::runtime_error(msg);
        }
    }

    // Close the tiff
    lt::TIFFClose(out);
}
