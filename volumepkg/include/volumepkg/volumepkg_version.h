//
// Created by Seth Parker on 4/28/15.
//
#pragma once

#include <string>
#include <unordered_map>
/**
 * @class VolumePkg_Version
 * These constants represent the various versions of the Volume package
 * Each version as a different library that may have varying information
 * conatined
 * or how the information is stored was changed between versions
 */
namespace volcart
{

// VersionDict's will hold possible metadata keys and their types

/**
 * This type sets the dictionary to be a map which contains 2 strings for each
 * entry.
 * The first string tells the user what is being saved in that place
 * The second string tells the user what type is saved.
 */
using Dictionary = std::unordered_map<std::string, std::string>;
/**
 * This type sets the Library to be a map which contains an integer and a
 * dictonary type
 * It essentially acts like a literal library, storing the dictionaries so they
 * can be easily found.
 * The integer acts as an index for each Dictonary
 * The Dictionary is the type stored there
 */
using Library = std::unordered_map<int, Dictionary>;

// clang-format off
/**
 * This is the dictionary that gives the data types for Volpkg 1
 */
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

/**
 * This is the dictionary that gives the data types for Volkpg 2
 */
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

/**
 * This is the dictionary that gives the data types for Volkpg 3
 */
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
/**
 * This library holds the Version Dictionaries and connects them to the possible
 * versions that a user might enter when creating a Volume Package
 */
const Library VersionLibrary = {{1, _1}, {2, _2}, {3, _3}};
}  // namespace volcart
