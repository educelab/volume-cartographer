// volcart::texturing::compositeTexture
// Generate a Texture object using one of the various filters in
// texturingUtils.h.
// Created by Seth Parker on 10/20/15.
#pragma once

#include "common/types/Texture.h"
#include "common/types/UVMap.h"
#include "common/vc_defines.h"
#include "common/types/VolumePkg.h"

#include "texturing/simpleUV.h"
#include "texturing/texturingUtils.h"

namespace volcart
{
namespace texturing
{
class compositeTexture
{
public:
    compositeTexture(
        ITKMesh::Pointer inputMesh,
        VolumePkg& volpkg,
        int output_w,
        int output_h,
        double radius,
        CompositeOption compositeMethod,
        DirectionOption compositeDirection);

    compositeTexture(
        ITKMesh::Pointer inputMesh,
        VolumePkg& volpkg,
        UVMap uvMap,
        double radius,
        CompositeOption method,
        DirectionOption direction);

    Texture texture() { return _texture; };
private:
    int _process();

    // Variables
    ITKMesh::Pointer _input;
    VolumePkg& _volpkg;
    int _width;
    int _height;
    double _radius;
    CompositeOption _method;
    DirectionOption _direction;

    UVMap _uvMap;
    Texture _texture;
};  // compositeTexture
}  // volcart
}  // texture
