#include "core/types/PerPixelMap.h"
#include "core/types/Exceptions.h"

using namespace volcart;
namespace fs = boost::filesystem;

///// Constructors /////
// Empty Map of width x height
PerPixelMap::PerPixelMap(int height, int width) : _height(height), _width(width)
{
    _map = cv::Mat_<cv::Vec6d>(height, width, cv::Vec6d(0, 0, 0, 0, 0, 0));
}

// Construct map from file
PerPixelMap::PerPixelMap(fs::path path) { read(path); }

///// Disk IO /////
void PerPixelMap::write(fs::path path)
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

void PerPixelMap::read(const fs::path& path)
{
    std::cerr << "volcart::PerPixelMap: Reading from file " << path.filename()
              << std::endl;
    cv::FileStorage file(path.string(), cv::FileStorage::READ);
    cv::FileNode map = file["PerPixelMapping"];

    // Read the header info
    _height = (int)map["rows"];
    _width = (int)map["cols"];

    // Fill the _map from the list of doubles
    _map = cv::Mat_<cv::Vec6d>(_height, _width, cv::Vec6d(0, 0, 0, 0, 0, 0));
    cv::Vec6d v;
    cv::FileNodeIterator dbl = map["data"].begin();

    // Make sure the size is as expected
    if (static_cast<int>(map["data"].size()) != _height * _width * 6) {
        throw IOException("size mismatch");
    }

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
