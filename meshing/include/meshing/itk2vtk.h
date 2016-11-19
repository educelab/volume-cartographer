#pragma once

#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

#include "core/vc_defines.h"

namespace volcart
{
namespace meshing
{
/**
 * @class itk2vtk
 * @author Seth Parker
 * @date 8/3/15
 *
 * @brief Convert from an ITKMesh to VTK PolyData.
 *
 * Copy vertices, vertex normals, and faces (cells) from input to output.
 *
 * @see  examples/src/itk2vtkExample.cpp
 *       meshing/test/itk2vtkTest.cpp
 *
 * @ingroup Meshing
 */
class itk2vtk
{
public:
    itk2vtk(ITKMesh::Pointer input, vtkSmartPointer<vtkPolyData> output);
};

/**
 * @class vtk2itk
 * @author Seth Parker
 * @date 8/3/15
 *
 * @brief Convert from a VTK PolyData to an ITKMesh.
 *
 * Copy vertices, vertex normals, and faces (cells) from input to output.
 *
 * @see  examples/src/itk2vtkExample.cpp
 *       meshing/test/itk2vtkTest.cpp
 *
 * @ingroup Meshing
 */
class vtk2itk
{
public:
    vtk2itk(vtkSmartPointer<vtkPolyData> input, ITKMesh::Pointer output);
};

/**
 * @class itk2itkQE
 * @author Seth Parker
 * @date 8/3/15
 *
 * @brief Convert from an ITKMesh to a QuadMesh (Quad Edge Mesh).
 *
 * Copy vertices, vertex normals, and faces (cells) from input to output.
 *
 * @ingroup Meshing
 */
class itk2itkQE
{
public:
    itk2itkQE(ITKMesh::Pointer input, volcart::QuadMesh::Pointer output);
};

/**
 * @class itkQE2itk
 * @author Seth Parker
 * @date 8/3/15
 *
 * @brief Convert from a QuadMesh (Quad Edge Mesh) to an ITKMesh.
 *
 * Copy vertices, vertex normals, and faces (cells) from input to output.
 *
 * @ingroup Meshing
 */
class itkQE2itk
{
public:
    itkQE2itk(volcart::QuadMesh::Pointer input, ITKMesh::Pointer output);
};

}  // namespace meshing
}  // namespace volcart
