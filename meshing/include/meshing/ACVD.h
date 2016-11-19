// ACVD
// This is a refactor of the ACVD implementation found in the ACVD.cxx example
// of https://github.com/valette/ACVD
// This function is essentially a wrapper around that functionality
//
// This implements the iterative process discussed in:
//      Valette, Sébastien, and Jean‐Marc Chassery. "Approximated centroidal
//      voronoi diagrams for uniform polygonal mesh coarsening." Computer
//      Graphics Forum. Vol. 23. No. 3. Blackwell Publishing, Inc, 2004.
//
// It iteratively loops over the mesh until the approximated centroidal voronoi
// diagrams for the mesh are approximately
// equivalent in area. It then takes the point on the mesh that is nearest to
// the centroid of each diagram as a new point
// in the resampled output mesh.

#pragma once

#include <vtkPolyData.h>
#include <vtkPolyDataNormals.h>
#include "vtkIsotropicDiscreteRemeshing.h"

#include "core/vc_defines.h"
#include "itk2vtk.h"

namespace volcart
{
namespace meshing
{
// inputMesh:       VTK PolyData to be remeshed
// numberOfSamples: Number of desired points in output mesh
// gradation:       Gradation parameter for ACVD
//                  Default: 0 for uniform gradation.
//                  Higher values give increasingly more importance to regions
//                  with high curvature
// consoleOutput:   Sets the graphics display
//                  0 : no display (default)
//                  1 : display
//                  2 : iterative display
// subsamplingThreshold: Higher values give better results but the input mesh
// will be subdivided more times
void ACVD(
    vtkPolyData* inputMesh,
    vtkPolyData* outputMesh,
    int numberOfSamples,
    float gradation = 0,
    int consoleOutput = 0,
    int subsamplingThreshold = 10);
}
}
