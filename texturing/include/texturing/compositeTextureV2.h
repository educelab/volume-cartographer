#pragma once

#include <opencv2/core.hpp>

#include "core/types/Texture.h"
#include "core/types/UVMap.h"
#include "core/types/VolumePkg.h"
#include "core/vc_defines.h"

#include "texturingUtils.h"

namespace volcart
{
namespace texturing
{

class compositeTextureV2
{
public:
    compositeTextureV2(
        ITKMesh::Pointer inputMesh,
        VolumePkg& volpkg,
        UVMap uvMap,
        double radius,
        size_t width,
        size_t height,
        CompositeOption method = CompositeOption::NonMaximumSuppression,
        DirectionOption direction = DirectionOption::Bidirectional);

    const volcart::Texture& texture() const { return _texture; };
    volcart::Texture& texture() { return _texture; };
private:
    int _process();

    // Variables
    ITKMesh::Pointer _input;
    VolumePkg& _volpkg;
    size_t _width;
    size_t _height;
    double _radius;
    CompositeOption _method;
    DirectionOption _direction;

    UVMap _uvMap;
    Texture _texture;
};
}
}
