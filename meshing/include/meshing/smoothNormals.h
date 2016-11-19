// smoothNormals.h
// Abigail Coleman June 2015
#pragma once

#include <fstream>
#include <iostream>

#include <vector>
#include <opencv2/opencv.hpp>

#include "core/vc_defines.h"
#include "meshing/deepCopy.h"

namespace volcart
{
namespace meshing
{
ITKMesh::Pointer smoothNormals(ITKMesh::Pointer input, double radius);
}
}
