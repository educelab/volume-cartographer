//
// Created by Seth Parker on 7/30/15.
//

#include <iostream>

#include "boost/filesystem.hpp"
#include "boost/program_options.hpp"
#include <boost/algorithm/string.hpp>

#include "vc_defines.h"
#include "volumepkg.h"

#ifndef VC_PACKAGER_H
#define VC_PACKAGER_H

// Struct to keep track of things we need to know about the slices
struct Slice {
    boost::filesystem::path path;
    unsigned long w, h;
    int depth;
    float min, max;
};

// Compare slices by their filepaths for sorting. Lexicographical comparison, but doesn't
// handle non-padded numbers (e.g. file8, file9, file10, file11)
bool SliceLess( const Slice& a, const Slice& b ) {
    std::string a_filename = boost::to_lower_copy<std::string>(a.path.filename().native());
    std::string b_filename = boost::to_lower_copy<std::string>(b.path.filename().native());
    return a_filename < b_filename;
}

#endif //VC_PACKAGER_H
