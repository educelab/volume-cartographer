//
// Created by Seth Parker on 3/18/16.
//

#include "core/types/PerPixelMap.h"

using namespace volcart;

///// Constructors /////
// Empty Map of width x height
PerPixelMap::PerPixelMap(size_t height, size_t width)
    : _height(height), _width(width)
{
    _initializeMap();
}

// Construct map from file
PerPixelMap::PerPixelMap(boost::filesystem::path path) { read(path); }

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
    return (_mask.at<unsigned char>(y, x) == 255);
}

// Initialize map
void PerPixelMap::_initializeMap()
{
    if (_height > 0 && _width > 0) {
        _map =
            cv::Mat_<cv::Vec6d>(_height, _width, cv::Vec6d(0, 0, 0, 0, 0, 0));
    }
}

///// Disk IO /////
void PerPixelMap::write(boost::filesystem::path path)
{

    // Ensure proper file extension
    path.replace_extension(".yml.gz");

    // Write to file
    std::cerr << "volcart::PerPixelMap: Writing to file " << path.filename()
              << std::endl;
    cv::FileStorage fs(path.string(), cv::FileStorage::WRITE);
    fs << "PerPixelMapping" << _map;
    fs.release();
}

void PerPixelMap::read(boost::filesystem::path path)
{

    std::cerr << "volcart::PerPixelMap: Reading from file " << path.filename()
              << std::endl;
    cv::FileStorage file(path.string(), cv::FileStorage::READ);
    cv::FileNode map = file["PerPixelMapping"];

    // Read the header info
    _height = static_cast<int>(map["rows"]);
    _width = static_cast<int>(map["cols"]);

    // Fill the _map from the list of doubles
    _map = cv::Mat_<cv::Vec6d>(_height, _width, cv::Vec6d(0, 0, 0, 0, 0, 0));
    cv::Vec6d v;
    cv::FileNodeIterator dbl = map["data"].begin();

    // Make sure the size is as expected
    // To-do: Throw exception if they don't match
    bool matches = map["data"].size() == _height * _width * 6;

    for (int y = 0; y < _height; ++y) {
        for (int x = 0; x < _width; ++x) {

            // Fill each cv::Vec6d
            for (int n = 0; n < 6; ++n, ++dbl) {
                v(n) = (double)(*dbl);
            }

            // Assign _map in the correct position
            _map(y, x) = v;

        }  // x
    }      // y

    file.release();
}
