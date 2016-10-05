//
// Created by Seth Parker on 4/28/15.
//
#pragma once

#include <string>
#include <unordered_map>

namespace volcart
{

// VersionDict's will hold possible metadata keys and their types
using Dictionary = std::unordered_map<std::string, std::string>;
using Library = std::unordered_map<int, Dictionary>;

// Version 1
// clang-format off
const Dictionary _1 =
        {
        {"volumepkg name",   "string"},
        {"version",          "int"},
        {"width",            "int"},
        {"height",           "int"},
        {"number of slices", "int"},
        {"slice location",   "string"},
        {"min",              "double"},
        {"max",              "double"},
        {"voxelsize",        "double"}
        };

// Version 2
const Dictionary _2 =
        {
        {"volumepkg name",   "string"},
        {"version",          "int"},
        {"width",            "int"},
        {"height",           "int"},
        {"number of slices", "int"},
        {"slice location",   "string"},
        {"min",              "double"},
        {"max",              "double"},
        {"voxelsize",        "double"},
        {"materialthickness","double"}
        };

// Version 3
const Dictionary _3 =
        {
        {"volumepkg name",   "string"},
        {"version",          "int"},
        {"width",            "int"},
        {"height",           "int"},
        {"number of slices", "int"},
        {"slice location",   "string"},
        {"min",              "double"},
        {"max",              "double"},
        {"voxelsize",        "double"},
        {"materialthickness","double"}
        };
// clang-format on

// Add the Version Dict's to a list of possible versions
const Library VersionLibrary = {{1, _1}, {2, _2}, {3, _3}};
}  // namespace volcart
