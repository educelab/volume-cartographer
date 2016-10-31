//
// Created by Hannah Hatch on 10/18/16.
//

#pragma once

#include <cmath>
#include <fstream>
#include <iostream>
#include <string.h>

#include "boost/algorithm/string/classification.hpp"
#include "boost/algorithm/string/split.hpp"
#include "boost/filesystem/path.hpp"
#include "common/vc_defines.h"
/**
 * @file PlyReader2.h
 * @author Hannah Hatch
 * @date 10/18/16
 * @brief Reads in a PLY file
 *
 * Reads in a PLY file by reading in the vertices and then the faces. It can
 * only parse point normals, points, and faces. It ignores any other features.
 *
 * It also only works for triangular meshes.
 *
 * @warning If faces aren't set, it will not hard fail but will give an error
 *
 * @ingroup VolCore
 *
 * @param path The location of the file you want to read in
 * @param mesh Where you want to store the mesh that's been read in
 */
namespace volcart {
    namespace io{
        bool PLYReader2(boost::filesystem::path path, volcart::ITKMesh::Pointer mesh);
    }
}
