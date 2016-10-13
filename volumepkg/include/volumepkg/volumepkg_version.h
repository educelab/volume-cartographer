/**
  @file   volumepkg_version.h
  @author Seth Parker
  @date   April 2015

  @brief  VolumePkg metadata templates.

  The objects in this file define the keys that can be expected
  in a VolumePkg's metadata JSON file, based on the VolumePkg version
  number.

  @newline
  A volcart::Dictionary defines a set of metadata keys and the associated types
  of the corresponding values (e.g. the key "version" should be interpreted as
  an `int`). A volcart::Library holds several such dictionaries: one dictionary
  for each unique version of VolumePkg.

  @newline
  volcart::VersionLibrary holds the metadata keys and expected value-types for
  every version of VolumePkg. When creating a new VolumePkg, these dictionaries
  are used to define the default keys that populate the JSON file. In the
  future, applications that use VolumePkg will be able to query the Library in
  order to determine what information can be expected in a particular version
  of a VolumePkg. In this way, developers will be able to maintain backwards
  compatibility with older datasets.

  @ingroup VolumePackage
 */

#pragma once

#include <string>
#include <unordered_map>

namespace volcart
{

/**
 * Holds a set of key-value pairs that map JSON metadata keys to the expected
 * type of the value. A template for the structure of a VolumePkg's metadata.
 */
using Dictionary = std::unordered_map<std::string, std::string>;

/**
 * Holds a set of key-value pairs that map version number keys to a specific
 * Dictionary of metadata mappings.
 */
using Library = std::unordered_map<int, Dictionary>;

// clang-format off
/** Metadata dictionary for VolumePkg v1. */
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

/** Metadata dictionary for VolumePkg v2. */
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

/** Metadata dictionary for VolumePkg v3. */
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

/**
 * Global Library used to store all template Dictionaries.
 */
const Library VersionLibrary = {{1, _1}, {2, _2}, {3, _3}};
}  // namespace volcart
