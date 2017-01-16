#include "core/types/PerPixelMap.h"
#include "core/types/Exceptions.h"

using namespace volcart;
namespace fs = boost::filesystem;

constexpr static size_t PPM_ELEMENT_SIZE = 6;

///// Metadata /////
void PerPixelMap::setDimensions(size_t h, size_t w)
{
    height_ = h;
    width_ = w;
    initialize_map_();
}

void PerPixelMap::setWidth(size_t w)
{
    width_ = w;
    initialize_map_();
}

void PerPixelMap::setHeight(size_t h)
{
    height_ = h;
    initialize_map_();
}

// Initialize map
void PerPixelMap::initialize_map_()
{
    if (height_ > 0 && width_ > 0) {
        map_ = cv::Mat_<cv::Vec6d>(height_, width_, {0, 0, 0, 0, 0, 0});
    }
}

///// Disk IO /////
void PerPixelMap::WritePPM(fs::path path, const PerPixelMap& map)
{
    // Ensure proper file extension
    path.replace_extension(".yml.gz");

    // Write to file
    std::cerr << "volcart::PerPixelMap: Writing to file " << path.filename()
              << std::endl;
    cv::FileStorage fs(path.string(), cv::FileStorage::WRITE);
    fs << "PerPixelMapping" << map.map_;
    fs.release();
}

PerPixelMap PerPixelMap::ReadPPM(const fs::path& path)
{
    std::cerr << "volcart::PerPixelMap: Reading from file " << path.filename()
              << std::endl;
    cv::FileStorage file(path.string(), cv::FileStorage::READ);
    cv::FileNode map = file["PerPixelMapping"];
    PerPixelMap ppm;

    // Read the header info
    ppm.height_ = static_cast<int>(map["rows"]);
    ppm.width_ = static_cast<int>(map["cols"]);

    // Initialize an empty map
    ppm.map_ = cv::Mat_<cv::Vec6d>(ppm.height_, ppm.width_, {0, 0, 0, 0, 0, 0});

    // Make sure the size is as expected
    if (map["data"].size() != ppm.height_ * ppm.width_ * 6) {
        auto msg = "Header dimensions do not match data dimensions";
        throw IOException(msg);
    }

    cv::Vec6d v;
    auto dbl = map["data"].begin();
    for (size_t y = 0; y < ppm.height_; ++y) {
        for (size_t x = 0; x < ppm.width_; ++x) {

            // Fill each cv::Vec6d
            for (size_t n = 0; n < PPM_ELEMENT_SIZE; ++n, ++dbl) {
                v(n) = static_cast<double>(*dbl);
            }

            // Assign _map in the correct position
            ppm.map_(y, x) = v;
        }
    }

    file.release();
    return ppm;
}
