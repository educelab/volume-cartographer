//
// Created by Seth Parker on 10/22/15.
//
#pragma once

#include "common/vc_defines.h"

#include <itkScaleTransform.h>
#include <itkTransformMeshFilter.h>

namespace volcart {
    namespace meshing {
            void scaleMesh( ITKMesh::Pointer input, ITKMesh::Pointer output, double scale_factor );
    } // meshing
} // volcart
