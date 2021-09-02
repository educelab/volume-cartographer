#pragma once

/** @file */

#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

#include "vc/core/types/ITKMesh.hpp"

namespace volcart::meshing
{
/**
 * @brief Convert from an ITKMesh to VTK PolyData.
 *
 * Copy vertices, vertex normals, and faces (cells) from input to output.
 *
 * @see  examples/src/ITK2VTKExample.cpp
 *       meshing/test/ITK2VTKTest.cpp
 *
 * @ingroup Meshing
 */
void ITK2VTK(ITKMesh::Pointer input, vtkSmartPointer<vtkPolyData> output);

/** @copydoc ITK2VTK */
auto ITK2VTK(ITKMesh::Pointer input) -> vtkSmartPointer<vtkPolyData>;

/**
 * @brief Convert from a VTK PolyData to an ITKMesh.
 *
 * Copy vertices, vertex normals, and faces (cells) from input to output.
 *
 * @see  examples/src/ITK2VTKExample.cpp
 *       meshing/test/ITK2VTKTest.cpp
 *
 * @ingroup Meshing
 */
void VTK2ITK(vtkSmartPointer<vtkPolyData> input, ITKMesh::Pointer output);

/** @copydoc VTK2ITK */
auto VTK2ITK(vtkSmartPointer<vtkPolyData> input) -> ITKMesh::Pointer;
}  // namespace volcart::meshing
