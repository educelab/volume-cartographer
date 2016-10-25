//
// Created by Seth Parker on 8/3/15.
//

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
 * @ingroup Meshing
 */
class itk2vtk
{
public:
    /**
     * @brief Converts from an ITK Mesh to VTK PolyData
     *
     *  This function copies over the points then converts point data
     *  into tuples (for point normals) and lastly copies cells.
     *   @see  examples/src/itk2vtkExample.cpp
     *         meshing/test/itk2vtkTest.cpp
     * @param input Pointer to an ITK mesh that needs to be converted
     * @param output Resulting pointer to vtkPolyData
     */
    itk2vtk(ITKMesh::Pointer input, vtkSmartPointer<vtkPolyData> output);
};

/**
 * @class vtk2itk
 * @author Seth Parker
 * @date 8/3/15
 *
 * @ingroup Meshing
 */
class vtk2itk
{
public:
    /**
     * @brief Converts from a VTK PolyData to an ITK Mesh
     *
     * The function copies over the points then converts the
     * tuples into point data (for point normals) and lastly
     * copies cells.
     *
     * @see examples/src/itk2vtkExample.cpp
     *      meshing/test/itk2vtkTest.cpp
     *
     * @param input Pointer to VTK Polydata that you want to convert
     * @param output Pointer to resutling ITK Mesh
     */
    vtk2itk(vtkSmartPointer<vtkPolyData> input, ITKMesh::Pointer output);
};
/**
 * @class itk2vtkQE
 * @author Seth Parker
 * @date 8/3/15
 *
 * @ingroup Meshing
 */
class itk2itkQE
{
public:
    /**
     * @brief Converts from an ITK Triangular Mesh to an ITK Quadrangle Mesh
     *
     * This class converts the face type of an ITK mesh from triangles to
     * quadrangles
     * This function copies the points from one mesh to the other. It
     * then sets the point ID's to a quadrangle cell.
     *
     * @see examples/src/itk2vtkExample.cpp
     *      meshing/test/itk2vtkTest.cpp
     *
     * @param input Pointer to ITK Triangular Mesh that you want to convert
     * @param output Pointer to ITK Quadrangle Mesh that results
     */
    itk2itkQE(ITKMesh::Pointer input, volcart::QuadMesh::Pointer output);
};
/**
 * @class itkQE2vtk
 * @author Seth Parker
 * @date 8/3/15
 *
 * @ingroup Meshing
 */
class itkQE2itk
{
public:
    /**
     * @brief Converts from an ITK Quadrangle Mesh to an ITK Triangular Mesh
     *
     * This class converts the face type of an ITK mesh from quadrangles to
     * triangles
     * This function copies the points from one mesh to the other. It
     * then sets the point ID's to a triangle cell
     *
     * @see examples/src/itk2vtkExample.cpp
     *      meshing/test/itk2vtkTest.cpp
     *
     * @param input Pointer to ITK Quadrangle Mesh that you want to convert
     * @param output Pointer to ITK Triangular Mesh that results
     */
    itkQE2itk(volcart::QuadMesh::Pointer input, ITKMesh::Pointer output);
};

}  // namespace meshing
}  // namespace volcart
