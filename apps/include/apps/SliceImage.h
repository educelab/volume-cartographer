#pragma once

#include <iostream>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <opencv2/core.hpp>

namespace volcart
{

class SliceImage
{
public:
    bool operator==(const SliceImage& b) const;
    bool operator!=(const SliceImage& b) const { return !operator==(b); }

    bool analyze();
    cv::Mat conformedImage();
    int width() { return _w; }
    int height() { return _h; }
    int depth() { return _depth; }
    double min() { return _min; }
    double max() { return _max; }
    bool needsConvert() { return _convert; }

    boost::filesystem::path path;

private:
    int _w, _h, _depth;
    double _min, _max;
    bool _convert = false;  // true if file needs to be converted
};

};  // namespace volcart

// Compare slices by their filepaths for sorting. Lexicographical comparison,
// but doesn't
// handle non-padded numbers (e.g. file8, file9, file10, file11)
inline bool SlicePathLessThan(
    const volcart::SliceImage& a, const volcart::SliceImage& b)
{
    std::string a_filename =
        boost::to_lower_copy<std::string>(a.path.filename().native());
    std::string b_filename =
        boost::to_lower_copy<std::string>(b.path.filename().native());
    return a_filename < b_filename;
}
