//
// Created by Seth Parker on 4/27/15.
//
#ifndef VC_PLYHELPER_H
#define VC_PLYHELPER_H

#include <cmath>
#include <fstream>
#include <iostream>

#include "common/vc_defines.h"
#include "boost/filesystem/path.hpp"

namespace volcart
{
namespace io
{
bool ply2itkmesh(boost::filesystem::path plyPath,
                 VC_MeshType::Pointer mesh,
                 int &width,
                 int &height);
}
}

#endif  // VC_PLYHELPER_H
