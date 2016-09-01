//
// Created by Seth Parker on 4/28/15.
//
#pragma once

#include <unordered_map>
#include <string>

namespace volcart
{

// VersionDict's will hold possible metadata keys and their types
using Dictionary = std::unordered_map<std::string, std::string>;
using Library = std::unordered_map<int, Dictionary>;            // Changed type from double to int

// Version 1
// clang-format off
const Dictionary _1_0 =
        {
        {"volumepkg name",   "string"},
        {"version",          "int"},    // Changed type from double to int
        {"width",            "int"},
        {"height",           "int"},
        {"number of slices", "int"},
        {"slice location",   "string"},
        {"min",              "double"},
        {"max",              "double"},
        {"voxelsize",        "double"}
        };

// Version 2
const Dictionary _2_0 =
        {
        {"volumepkg name",   "string"},
        {"version",          "int"},    // Changed type from double to int
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
const Library VersionLibrary = {{1, _1_0},      // Changed type from double to int
                                {2, _2_0}};
// clang-format on
}  // namespace volcart
