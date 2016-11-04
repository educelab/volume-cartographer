// smoothNormals.h
// Abigail Coleman June 2015
#pragma once

#include <fstream>
#include <iostream>
#include <vector>

#include "common/vc_defines.h"
#include "meshing/deepCopy.h"

namespace volcart
{
namespace meshing
{
ITKMesh::Pointer smoothNormals(ITKMesh::Pointer input, double radius);
}
}
