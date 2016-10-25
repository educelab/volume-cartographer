//
// Created by Media Team on 8/12/15.
//
#pragma once

/**
 * @class rayTrace
 * @author Media Team
 * @date 8/12/15
 *
 */
#include <vector>
#include <opencv2/opencv.hpp>
#include "core/vc_defines.h"

namespace volcart
{
namespace meshing
{
/**
 * @brief Creates a UV Map of the mesh for texturing
 *
 * This class uses vtk's OBBTree to test a ray's intersection with the
 * faces/cells/triangles in the mesh.
 *
 * @param itkMesh Mesh that you want to trace
 * @param aTraceDir Direction you want to move (1 for clockwise)
 * @param width Width of the ray
 * @param height Height of the ray
 * @param uvMap Map of the UV coordinates found by the ray
 * @return vector of CV points that tell where the intersections are
 */
std::vector<cv::Vec6f> rayTrace(
    ITKMesh::Pointer itkMesh,
    int aTraceDir,
    int width,
    int height,
    std::map<int, cv::Vec2d> uvMap);
}
}
