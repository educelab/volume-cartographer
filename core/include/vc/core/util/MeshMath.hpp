/**
 * @file MeshMath.hpp
 * @author Seth Parker
 * @date 6/7/16
 *
 * @brief Utility functions for calculations on meshes
 *
 * @ingroup Util
 */
#pragma once

#include "vc/core/types/ITKMesh.hpp"

namespace volcart
{
namespace meshmath
{
/**
 * @brief Calculate the surface area of an ITKMesh
 *
 * Uses a version of Heron's formula that is stable for small angles.
 */
double SurfaceArea(const ITKMesh::Pointer& input);
}
}
