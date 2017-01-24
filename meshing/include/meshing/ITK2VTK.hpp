#pragma once

#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

#include "core/vc_defines.hpp"

namespace volcart
{
namespace meshing
{
/**
 * @class ITK2VTK
 * @author Seth Parker
 * @date 8/3/15
 *
 * @brief Convert from an ITKMesh to VTK PolyData.
 *
 * Copy vertices, vertex normals, and faces (cells) from input to output.
 *
 * @see  examples/src/ITK2VTKExample.cpp
 *       meshing/test/ITK2VTKTest.cpp
 *
 * @ingroup Meshing
 */
class ITK2VTK
{
public:
    ITK2VTK(ITKMesh::Pointer input, vtkSmartPointer<vtkPolyData> output);
};

/**
 * @class VTK2ITK
 * @author Seth Parker
 * @date 8/3/15
 *
 * @brief Convert from a VTK PolyData to an ITKMesh.
 *
 * Copy vertices, vertex normals, and faces (cells) from input to output.
 *
 * @see  examples/src/ITK2VTKExample.cpp
 *       meshing/test/ITK2VTKTest.cpp
 *
 * @ingroup Meshing
 */
class VTK2ITK
{
public:
    VTK2ITK(vtkSmartPointer<vtkPolyData> input, ITKMesh::Pointer output);
};

/**
 * @class ITK2ITKQE
 * @author Seth Parker
 * @date 8/3/15
 *
 * @brief Convert from an ITKMesh to a QuadMesh (Quad Edge Mesh).
 *
 * Copy vertices, vertex normals, and faces (cells) from input to output.
 *
 * @ingroup Meshing
 */
class ITK2ITKQE
{
public:
    ITK2ITKQE(ITKMesh::Pointer input, volcart::QuadMesh::Pointer output);
};

/**
 * @class ITKQE2ITK
 * @author Seth Parker
 * @date 8/3/15
 *
 * @brief Convert from a QuadMesh (Quad Edge Mesh) to an ITKMesh.
 *
 * Copy vertices, vertex normals, and faces (cells) from input to output.
 *
 * @ingroup Meshing
 */
class ITKQE2ITK
{
public:
    ITKQE2ITK(volcart::QuadMesh::Pointer input, ITKMesh::Pointer output);
};
}
}
