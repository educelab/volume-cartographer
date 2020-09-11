#pragma once

#include <string>

namespace volcart
{

/** Provides programmatic access to codebase information */
struct ProjectInfo {
    /** Get the library name */
    static std::string Name();
    /** Get the library version as a Major.Minor.Patch string */
    static std::string VersionString();
    /** Get the library name and version string */
    static std::string NameAndVersion();
    /** Get the library Major version number */
    static uint32_t VersionMajor();
    /** Get the library Minor version number */
    static uint32_t VersionMinor();
    /** Get the library Patch version number */
    static uint32_t VersionPatch();
    /** Get the git repository URL */
    static std::string RepositoryURL();
    /** Get the full hash for the current git commit */
    static std::string RepositoryHash();
    /** Get the short hash for the current git commit */
    static std::string RepositoryShortHash();
};

}  // namespace volcart