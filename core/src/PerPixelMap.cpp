//
// Created by Seth Parker on 3/18/16.
//

#include "core/types/PerPixelMap.h"
#include "core/types/Exceptions.h"

using namespace volcart;
namespace fs = boost::filesystem;
<<<<<<< HEAD
=======

constexpr static size_t PPM_ELEMENT_SIZE = 6;
>>>>>>> develop

///// Constructors /////
// Empty Map of width x height
PerPixelMap::PerPixelMap(size_t height, size_t width)
    : _height{height}, _width{width}
{
<<<<<<< HEAD
    _map = cv::Mat_<cv::Vec6d>(height, width, cv::Vec6d{0, 0, 0, 0, 0, 0});
}

// Construct map from file
PerPixelMap::PerPixelMap(fs::path path) { read(path); }

///// Disk IO /////
void PerPixelMap::write(fs::path path)
{
=======
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
>>>>>>> develop
    // Ensure proper file extension
    path.replace_extension(".yml.gz");

    // Write to file
    std::cerr << "volcart::PerPixelMap: Writing to file " << path.filename()
              << std::endl;
    cv::FileStorage fs(path.string(), cv::FileStorage::WRITE);
    fs << "PerPixelMapping" << map._map;
    fs.release();
}

<<<<<<< HEAD
void PerPixelMap::read(fs::path path)
=======
PerPixelMap PerPixelMap::ReadPPM(fs::path path)
>>>>>>> develop
{
    std::cerr << "volcart::PerPixelMap: Reading from file " << path.filename()
              << std::endl;
    cv::FileStorage file(path.string(), cv::FileStorage::READ);
    cv::FileNode map = file["PerPixelMapping"];
    PerPixelMap ppm;

    // Read the header info
<<<<<<< HEAD
    _height = static_cast<int>(map["rows"]);
    _width = static_cast<int>(map["cols"]);

    // Fill the _map from the list of doubles
    _map = cv::Mat_<cv::Vec6d>(_height, _width, cv::Vec6d(0, 0, 0, 0, 0, 0));

    // Make sure the size is as expected
    if (static_cast<int>(map["data"].size()) != _height * _width * 6) {
        throw std::logic_error("Inconsistent size");
    }

    cv::Vec6d v;
    cv::FileNodeIterator dbl = map["data"].begin();
    for (int y = 0; y < _height; ++y) {
        for (int x = 0; x < _width; ++x) {

            // Fill each cv::Vec6d
            for (int n = 0; n < 6; ++n, ++dbl) {
=======
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
>>>>>>> develop
                v(n) = static_cast<double>(*dbl);
            }

            // Assign _map in the correct position
<<<<<<< HEAD
            _map(y, x) = v;
=======
            ppm._map(y, x) = v;
>>>>>>> develop
        }
    }

    file.release();
    return ppm;
}
