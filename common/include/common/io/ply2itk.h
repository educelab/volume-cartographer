//
// Created by Seth Parker on 4/27/15.
//
#pragma once

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
                 MeshType::Pointer mesh,
                 int &width,
                 int &height);
}
}
