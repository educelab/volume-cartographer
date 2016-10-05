//
// Created by Seth Parker on 10/22/15.
//
// Simple UV Creation
// This function is meant to aid in the creation of simple ("dumb") UV maps for meshes created from ordered PCDs.
// "Ordered meshes" are meshes with an equal number of points in each intersected z-index such that the mesh has
// something resembling a 2D width and height.
//
// THIS FUNCTION HAS LIMITED USE AND SHOULD BE USED WITH CAUTION
#pragma once

#include "common/vc_defines.h"
#include "common/types/UVMap.h"

namespace volcart {
    namespace texturing {
        volcart::UVMap simpleUV(ITKMesh::Pointer mesh, int width, int height);
    }; // texturing
}; // volcart
