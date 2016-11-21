//
// Created by Seth Parker on 4/27/15.
//
#pragma once

#include <cmath>
#include <fstream>
#include <iostream>

#include "boost/filesystem/path.hpp"
#include "core/vc_defines.h"

namespace volcart
{
namespace io
{
bool PLYReader(boost::filesystem::path path, ITKMesh::Pointer mesh);
}
}
