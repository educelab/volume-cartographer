//
// Created by Seth Parker on 4/28/15.
//
#ifndef VC_VOLUMEPKG_VERSION_H
#define VC_VOLUMEPKG_VERSION_H

#include <unordered_map>
#include <string>

namespace VolCart {

    // VersionDict's will hold possible metadata keys and their types
    typedef std::unordered_map < std::string, std::string > VersionDict;

    // Version 1.0
    const VersionDict _1_0 =
            {
            {"volumepkg name",   "string"},
            {"version",          "double"},
            {"width",            "int"},
            {"height",           "int"},
            {"number of slices", "int"},
            {"slice location",   "string"},
            {"min",              "double"},
            {"max",              "double"},
            {"voxelsize",        "double"}
            };

    // Version 2.0
    const VersionDict _2_0 =
            {
            {"volumepkg name",   "string"},
            {"version",          "double"},
            {"width",            "int"},
            {"height",           "int"},
            {"number of slices", "int"},
            {"slice location",   "string"},
            {"min",              "double"},
            {"max",              "double"},
            {"voxelsize",        "double"},
            {"materialthickness","double"}
            };

    // Add the Version Dict's to a list of possible versions
    const std::unordered_map < double, VersionDict > versionsList = {{1.0, _1_0},
                                                                     {2.0, _2_0}};;
}

#endif //VC_VOLUMEPKG_VERSION_H
