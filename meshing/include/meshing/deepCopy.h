//
// Created by Seth Parker on 12/21/15.
//
#pragma once

#include "core/vc_defines.h"

namespace volcart
{
namespace meshing
{
class deepCopy
{
public:
    deepCopy(ITKMesh::Pointer input, ITKMesh::Pointer output);
};  // deepCopy
}  // meshing
}  // volcart
