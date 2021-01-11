#pragma once

/** @file */

#include "vc/core/types/ITKMesh.hpp"

namespace volcart
{
namespace meshing
{
/**
 * @author Abigail Coleman
 * @date June 2015
 *
 * @brief Smooth vertex normals within a specified radius.
 *
 * Uses ITK's neighborhood locator to get the list of neighboring vertices
 * within the provided spherical radius. Returns a DeepCopy of the
 * original mesh, with smoothed vertex normals.
 *
 * @ingroup Meshing
 *
 * @param radius Size of the spherical neighborhood
 */
ITKMesh::Pointer SmoothNormals(const ITKMesh::Pointer& input, double radius);
}  // namespace meshing
}  // namespace volcart
