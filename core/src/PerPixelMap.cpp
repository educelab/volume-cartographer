//
// Created by Seth Parker on 3/18/16.
//

#include "core/types/PerPixelMap.h"
#include "core/types/Exceptions.h"

using namespace volcart;
namespace fs = boost::filesystem;

constexpr static size_t PPM_ELEMENT_SIZE = 6;

///// Constructors /////
// Empty Map of width x height
PerPixelMap::PerPixelMap(size_t height, size_t width)
    : _width{height}, _height{width}
{
    _initializeMap();
}

///// Metadata /////
void PerPixelMap::setDimensions(size_t h, size_t w)
{
    _height = h;
    _width = w;
    _initializeMap();
}

void PerPixelMap::setWidth(size_t w)
{
    _width = w;
    _initializeMap();
}

void PerPixelMap::setHeight(size_t h)
{
    _height = h;
    _initializeMap();
}

bool PerPixelMap::hasMapping(size_t y, size_t x)
{
    return _mask.at<uint8_t>(y, x) == 255;
}

// Initialize map
void PerPixelMap::_initializeMap()
{
    if (_height > 0 && _width > 0) {
        _map = cv::Mat_<cv::Vec6d>(_height, _width, {0, 0, 0, 0, 0, 0});
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
    fs << "PerPixelMapping" << map._map;
    fs.release();
}

PerPixelMap PerPixelMap::ReadPPM(fs::path path)
{
    std::cerr << "volcart::PerPixelMap: Reading from file " << path.filename()
              << std::endl;
    cv::FileStorage file(path.string(), cv::FileStorage::READ);
    cv::FileNode map = file["PerPixelMapping"];
    PerPixelMap ppm;

    // Read the header info
    ppm._height = static_cast<int>(map["rows"]);
    ppm._width = static_cast<int>(map["cols"]);

    // Initialize an empty map
    ppm._map = cv::Mat_<cv::Vec6d>(ppm._height, ppm._width, {0, 0, 0, 0, 0, 0});

    // Make sure the size is as expected
    if (map["data"].size() != ppm._height * ppm._width * 6) {
        auto msg = "Header dimensions do not match data dimensions";
        throw IOException(msg);
    }

    cv::Vec6d v;
    auto dbl = map["data"].begin();
    for (size_t y = 0; y < ppm._height; ++y) {
        for (size_t x = 0; x < ppm._width; ++x) {

            // Fill each cv::Vec6d
            for (size_t n = 0; n < PPM_ELEMENT_SIZE; ++n, ++dbl) {
                v(n) = static_cast<double>(*dbl);
            }

            // Assign _map in the correct position
            ppm._map(y, x) = v;
        }
    }

    file.release();
    return ppm;
}
