//
// Created by Seth Parker on 8/3/15.
//

#ifndef VC_ITK2VTK_H
#define VC_ITK2VTK_H

#include <vtkCellData.h>
#include <vtkCellArray.h>
#include <vtkDoubleArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkSmartPointer.h>

#include "common/vc_defines.h"

namespace volcart {
    namespace meshing {

        class itk2vtk {
        public:
            itk2vtk( VC_MeshType::Pointer input, vtkPolyData* output );
        };

        class vtk2itk {
        public:
            vtk2itk( vtkPolyData* input, VC_MeshType::Pointer output );
        };

        class itk2itkQE {
        public:
            itk2itkQE( VC_MeshType::Pointer input, volcart::QuadMesh::Pointer output );
        };

        class itkQE2itk {
        public:
            itkQE2itk( volcart::QuadMesh::Pointer input, VC_MeshType::Pointer output );
        };

    } // namespace meshing
} // namespace volcart

#endif //VC_ITK2VTK_H
