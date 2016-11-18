/**
 * @file smoothNormals.h
 * @brief Smooth vertex normals within a specified radius.
 */
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
/**
 * @author Abigail Coleman
 * @date June 2015
 *
 * @brief Smooth vertex normals within a specified radius.
 *
 * Uses ITK's neighborhood locator to get the list of neighboring vertices
 * within the provided spherical radius. Returns a deepCopy of the
 * original mesh, with smoothed vertex normals.
 *
 * @ingroup Meshing
 *
 * @param radius Size of the spherical neighborhood
 */
ITKMesh::Pointer smoothNormals(ITKMesh::Pointer input, double radius);
}
}
