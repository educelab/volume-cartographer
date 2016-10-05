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
bool PLYReader(boost::filesystem::path path, ITKMesh::Pointer mesh);
}
}
