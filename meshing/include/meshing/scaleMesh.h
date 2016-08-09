//
// Created by Seth Parker on 10/22/15.
//

#ifndef VC_SCALEMESH_H
#define VC_SCALEMESH_H

#include "common/vc_defines.h"

#include <itkScaleTransform.h>
#include <itkTransformMeshFilter.h>

namespace volcart {
    namespace meshing {
            void scaleMesh( VC_MeshType::Pointer input, VC_MeshType::Pointer output, double scale_factor );
    } // meshing
} // volcart

#endif //VC_SCALEMESH_H
