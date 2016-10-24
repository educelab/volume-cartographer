
#pragma once

#include <vtkPolyData.h>
#include <vtkPolyDataNormals.h>
#include "vtkIsotropicDiscreteRemeshing.h"

#include "core/vc_defines.h"
#include "itk2vtk.h"

/**
 * @class ACVD
 * @brief This is a refactor of the ACVD implementation found in the ACVD.cxx
 * example
 *        of https://github.com/valette/ACVD
 *
 * @newline
 * This function is essentially a wrapper around that functionality
 *
 * @newline
 * This implements the iterative process discussed in:
 *      Valette, Sébastien, and Jean‐Marc Chassery. "Approximated centroidal
 *      voronoi diagrams for uniform polygonal mesh coarsening." Computer
 *      Graphics Forum. Vol. 23. No. 3. Blackwell Publishing, Inc, 2004.
 *
 * @newline
 * It iteratively loops over the mesh until the approximated centroidal voronoi
 * diagrams for the mesh are approximately
 * equivalent in area. It then takes the point on the mesh that is nearest to
 * the centroid of each diagram as a new point
 * in the resampled output mesh.
 *
 * @ingroup Meshing
 *
 */

namespace volcart
{
namespace meshing
{
/**
 * @brief Resamples a surface using the ACVD algorithm
 *
 * Reduces the number of points and faces in a mesh by using the ACVD algorithm
 * described above
 *
 * @param inputMesh VTK PolyData to be remeshed
 * @param outputMesh VTK PolyData that has the desired number of points
 * @param numberOfSamples Number of desired points in output mesh
 * @param gradation Gradation parameter for ACVD
 *                  Default: 0 for uniform gradation.
 *                  Higher values give increasingly more importance to regions
 *                  with high curvature
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
