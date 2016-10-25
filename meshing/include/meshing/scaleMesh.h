//
// Created by Seth Parker on 10/22/15.
//
#pragma once
/**
 * @class scaleMesh
 * @author Seth Parker
 * @date 10/22/15
 */
#include "core/vc_defines.h"

#include <itkScaleTransform.h>
#include <itkTransformMeshFilter.h>

namespace volcart
{
namespace meshing
{
/**
 * @brief Scales the mesh down by a specifed factor
 *
 * This function scales the mesh uniformly by applying
 * an ITK filter.
 * @param input Pointer to an ITKMesh that needs to be scaled
 * @param output Pointer to an ITKMesh that has been scaled
 * @param scale_factor Factor by which the mesh should be scaled up or down
 */
void scaleMesh(
    ITKMesh::Pointer input, ITKMesh::Pointer output, double scale_factor);
}  // meshing
}  // volcart
