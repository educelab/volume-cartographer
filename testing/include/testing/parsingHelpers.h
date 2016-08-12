//
// Created by Ryan Taber on 11/19/15.
//

#ifndef VC_PARSINGHELPERS_H
#define VC_PARSINGHELPERS_H

#include "common/vc_defines.h"
#include <boost/filesystem/path.hpp>

/*
 * Purpose of File:
 *     - provides helpful parsing methods that are used by multiple
 *       testing files when reading in mesh or point cloud data
 */

namespace volcart {
namespace testing {

class ParsingHelpers{

public:
    static void parsePlyFile(const boost::filesystem::path& filepath, std::vector<VC_Vertex> &verts, std::vector<VC_Cell> &faces);
    static void parseObjFile(const boost::filesystem::path& filepath, std::vector<VC_Vertex> &points, std::vector<VC_Cell> &cells);

private:
    static std::vector<std::string> split_string(std::string input);
};


}
}
#endif //VC_PARSINGHELPERS_H
