#pragma once

/**
 * @file
 *
 * Provides helpful parsing methods that are used by multiple testing files
 * when reading in mesh or point cloud data
 */

#include <boost/filesystem/path.hpp>

#include "vc/core/types/SimpleMesh.hpp"

namespace volcart
{
namespace testing
{

class ParsingHelpers
{

public:
    static void ParsePLYFile(
        const boost::filesystem::path& filepath,
        std::vector<SimpleMesh::Vertex>& verts,
        std::vector<SimpleMesh::Cell>& faces);
    static void ParseOBJFile(
        const boost::filesystem::path& filepath,
        std::vector<SimpleMesh::Vertex>& points,
        std::vector<SimpleMesh::Cell>& cells);

private:
    static std::vector<std::string> SplitString(std::string input);
};
}  // namespace testing
}  // namespace volcart
