// smoothNormals.h
// Abigail Coleman June 2015
#pragma once
/**
 * @class smoothNormals
 * @author Abigail Coleman
 * @date June 2015
 */
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
 * @brief Calculates normals so they are similar in an area
 *
 * This function smooths the normals so they are more similar
 * so that patterns can be more easily seen in an area. This
 * is done by recalculating the normals in a neighborhood of pixels.
 * The bigger the neighborhood, the more generalized the normals will
 * be.
 *
 * @param input Pointer to an ITK Mesh whose normals should be smoothed
 * @param radius Indicates how big of a neighborhood you want
 * @return Pointer to an ITK Mesh with smoothed normals
 */
ITKMesh::Pointer smoothNormals(ITKMesh::Pointer input, double radius);
}
}
