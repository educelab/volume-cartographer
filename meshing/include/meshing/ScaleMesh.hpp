/**
 * @file ScaleMesh.h
 * @brief Scale an ITKMesh by a linear scale factor.
 */
#pragma once

#include "core/vc_defines.hpp"

#include <itkScaleTransform.h>
#include <itkTransformMeshFilter.h>

namespace volcart
{
namespace meshing
{
/**
 * @author Seth Parker
 * @date 10/22/15
 *
 * @brief Scale an ITKMesh by a linear scale factor.
 *
 * Uniform scaling of an ITKMesh by a linear scale factor (not an area scale
 * factor).
 *
 * @warning Output parameter should point to an empty ITKMesh. This function
 * will not initialize a new ITKMesh::Pointer.
 *
 * @ingroup Meshing
 */
void ScaleMesh(
    const ITKMesh::Pointer& input,
    const ITKMesh::Pointer& output,
    double scaleFactor);
}
}
