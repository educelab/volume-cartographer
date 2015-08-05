//
// Created by Seth Parker on 7/31/15.
//

#ifndef VC_SLICE_H
#define VC_SLICE_H

#include <iostream>

#include "boost/filesystem.hpp"
#include "boost/algorithm/string.hpp"
#include "opencv2/opencv.hpp"

namespace volcart {

    class Slice {
    public:

        bool operator==(const Slice& b) const;
        bool operator!=(const Slice& b) const { return !operator==(b); }

        bool analyze();
        cv::Mat image();
        int width() { return _w; };
        int height() { return _h; };
        int depth() { return _depth; };
        double min() { return _min; };
        double max() { return _max; };
        bool needsConvert() { return _convert; };

        boost::filesystem::path path;

    private:
        int _w, _h, _depth;
        double _min, _max;
        bool _convert = false; // true if file needs to be converted
    };

}; // namespace volcart

// Compare slices by their filepaths for sorting. Lexicographical comparison, but doesn't
// handle non-padded numbers (e.g. file8, file9, file10, file11)
inline bool SliceLess( const volcart::Slice& a, const volcart::Slice& b ) {
    std::string a_filename = boost::to_lower_copy<std::string>(a.path.filename().native());
    std::string b_filename = boost::to_lower_copy<std::string>(b.path.filename().native());
    return a_filename < b_filename;
};


#endif //VC_SLICE_H
