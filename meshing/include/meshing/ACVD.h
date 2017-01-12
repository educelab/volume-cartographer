/**
 * @file ACVD.h
 * @brief Mesh resampling using Approximated Centroidal Voronoi Diagrams.
 */
#pragma once

#include <vtkPolyData.h>

#include "core/vc_defines.h"

namespace volcart
{
namespace meshing
{
/**
 * @fn void ACVD( vtkPolyData* inputMesh, vtkPolyData* outputMesh,
 * int numberOfSamples, float gradation, int consoleOutput,
 * int subsamplingThreshold)
 * @brief Mesh resampling using Approximated Centroidal Voronoi Diagrams.
 *
 * This is a wrapper around the ACVD implementation found in the ACVD.cxx
 * example of <a href="https://github.com/valette/ACVD">ACVD</a>. This
 * implements the iterative process discussed in:
 *      Valette, Sébastien, and Jean‐Marc Chassery. "Approximated centroidal
 *      voronoi diagrams for uniform polygonal mesh coarsening." Computer
 *      Graphics Forum. Vol. 23. No. 3. Blackwell Publishing, Inc, 2004.
 *
 * Iteratively loops over the mesh until the approximated centroidal voronoi
 * diagrams for the mesh are approximately equivalent in area. It then takes
 * the point on the mesh that is nearest to the centroid of each diagram as a
 * new vertex in the resampled output mesh.
 *
 * @ingroup Meshing
 *
 * @param inputMesh VTK PolyData to be remeshed
 * @param outputMesh Resampled VTK PolyData
 * @param numberOfSamples Number of desired points in output mesh
 * @param gradation Gradation parameter for ACVD
 *                  Default: 0 for uniform gradation. Higher values give
 *                  increasingly more importance to regions with high curvature
 * @param consoleOutput Sets the graphics display
 *                      0 : no display (default)
 *                      1 : display
 *                      2 : iterative display
 * @param subsamplingThreshold Higher values give better results but the input
 *                             mesh will be subdivided more times
 */
void ACVD(
    vtkPolyData* inputMesh,
    vtkPolyData* outputMesh,
    int numberOfSamples,
    float gradation = 0,
    int consoleOutput = 0,
    int subsamplingThreshold = 10);
}
}
