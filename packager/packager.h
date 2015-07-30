//
// Created by Seth Parker on 7/30/15.
//

#include <iostream>

#include "boost/filesystem.hpp"
#include "boost/program_options.hpp"
#include <boost/algorithm/string.hpp>

#ifndef VC_PACKAGER_H
#define VC_PACKAGER_H

struct Slice {
    boost::filesystem::path path;
    unsigned long w, h;
    int depth;
    float min, max;
};

// compare slices by their filepaths. This attempts lexicographical
bool SliceLess( const Slice& a, const Slice& b ) {
    std::string a_filename = boost::to_lower_copy<std::string>(a.path.filename().native());
    std::string b_filename = boost::to_lower_copy<std::string>(b.path.filename().native());
    return a_filename < b_filename;
}

#endif //VC_PACKAGER_H
