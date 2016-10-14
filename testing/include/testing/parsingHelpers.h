//
// Created by Ryan Taber on 11/19/15.
//
#pragma once

#include <boost/filesystem/path.hpp>
#include "common/vc_defines.h"

/*
 * Purpose of File:
 *     - provides helpful parsing methods that are used by multiple
 *       testing files when reading in mesh or point cloud data
 */

namespace volcart
{
namespace testing
{

class ParsingHelpers
{

public:
    static void parsePlyFile(
        const boost::filesystem::path& filepath,
        std::vector<Vertex>& verts,
        std::vector<Cell>& faces);
    static void parseObjFile(
        const boost::filesystem::path& filepath,
        std::vector<Vertex>& points,
        std::vector<Cell>& cells);

private:
    static std::vector<std::string> split_string(std::string input);
};
}
}
