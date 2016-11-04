#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "common/vc_defines.h"

namespace volcart
{
namespace meshing
{
// Using vtk's OBBTree to test a ray's intersection with the
// faces/cells/triangles in the mesh
std::vector<cv::Vec6f> rayTrace(
    ITKMesh::Pointer itkMesh,
    int aTraceDir,
    int width,
    int height,
    std::map<int, cv::Vec2d> uvMap);
}
}
