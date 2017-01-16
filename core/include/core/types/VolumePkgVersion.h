/**
  @file   VolumePkgVersion.h
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
  volcart::VERSION_LIBRARY holds the metadata keys and expected value-types for
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
 * Types stored in `Dictionaries`
 */
enum class Type { STRING, INT, DOUBLE };

/**
 * Holds a set of key-value pairs that map JSON metadata keys to the expected
 * type of the value. A template for the structure of a VolumePkg's metadata.
 */
using Dictionary = std::unordered_map<std::string, Type>;

/**
 * Holds a set of key-value pairs that map version number keys to a specific
 * Dictionary of metadata mappings.
 */
using Library = std::unordered_map<int, Dictionary>;

// clang-format off
/** Metadata dictionary for VolumePkg v1. */
const Dictionary V1 =
        {
        {"volumepkg name",   Type::STRING},
        {"version",          Type::INT},
        {"width",            Type::INT},
        {"height",           Type::INT},
        {"number of slices", Type::INT},
        {"slice location",   Type::STRING},
        {"min",              Type::DOUBLE},
        {"max",              Type::DOUBLE},
        {"voxelsize",        Type::DOUBLE}
        };

/** Metadata dictionary for VolumePkg v2. */
const Dictionary V2 =
        {
        {"volumepkg name",   Type::STRING},
        {"version",          Type::INT},
        {"width",            Type::INT},
        {"height",           Type::INT},
        {"number of slices", Type::INT},
        {"slice location",   Type::STRING},
        {"min",              Type::DOUBLE},
        {"max",              Type::DOUBLE},
        {"voxelsize",        Type::DOUBLE},
        {"materialthickness",Type::DOUBLE}
        };

/** Metadata dictionary for VolumePkg v3. */
const Dictionary V3 =
        {
        {"volumepkg name",   Type::STRING},
        {"version",          Type::INT},
        {"width",            Type::INT},
        {"height",           Type::INT},
        {"number of slices", Type::INT},
        {"slice location",   Type::STRING},
        {"min",              Type::DOUBLE},
        {"max",              Type::DOUBLE},
        {"voxelsize",        Type::DOUBLE},
        {"materialthickness",Type::DOUBLE}
        };
// clang-format on

/**
 * Global Library used to store all template Dictionaries.
 */
const Library VERSION_LIBRARY = {{1, V1}, {2, V2}, {3, V3}};
}
