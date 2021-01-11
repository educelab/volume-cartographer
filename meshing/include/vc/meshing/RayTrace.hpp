#pragma once

/** @file */

#include <map>
#include <vector>

#include <opencv2/opencv.hpp>

#include "vc/core/types/ITKMesh.hpp"

namespace volcart
{
namespace meshing
{

/**
 * @author Mike Roup, Abigail Coleman, Seth Parker
 * @date 8/12/15
 *
 * @brief Mesh resampling using ray tracing.
 *
 * Uses ray tracing and intersection to compute ordered intersection points on
 * the surface of an ITKMesh.
 *
 * @warning Don't use this function for anything.
 *
 * @ingroup Meshing
 *
 * @param itkMesh Mesh to resample
 * @param aTraceDir Rotation direction (1 for clockwise)
 * @param width Number of rotational samples
 * @param height Number of vertical samples
 * @param uvMap Parameterized UV position for each point in the return vector
 * @return Vector of intersection points and the normal to that point
 */
std::vector<cv::Vec6f> RayTrace(
    const ITKMesh::Pointer& itkMesh,
    int aTraceDir,
    int width,
    int height,
    std::map<int, cv::Vec2d>& uvMap);
}  // namespace meshing
}  // namespace volcart
