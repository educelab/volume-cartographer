// volcart::texturing::CompositeTexture
// Generate a Texture object using one of the various filters in
// texturingUtils.h.
// Created by Seth Parker on 10/20/15.
#pragma once

#include "vc/core/types/Texture.hpp"
#include "vc/core/types/UVMap.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/vc_defines.hpp"
#include "texturing/SimpleUV.hpp"
#include "texturing/TexturingUtils.hpp"

namespace volcart
{
namespace texturing
{

class CompositeTexture
{
public:
    CompositeTexture(
        ITKMesh::Pointer inputMesh,
        VolumePkg& volpkg,
        int outputWidth,
        int outputHeight,
        double radius,
        CompositeOption compositeMethod,
        DirectionOption compositeDirection);

    CompositeTexture(
        ITKMesh::Pointer inputMesh,
        VolumePkg& volpkg,
        UVMap uvMap,
        double radius,
        CompositeOption method,
        DirectionOption direction);

    Texture texture() { return texture_; }

private:
    int process_();

    // Variables
    ITKMesh::Pointer input_;
    VolumePkg& volpkg_;
    int width_;
    int height_;
    double radius_;
    CompositeOption method_;
    DirectionOption direction_;

    UVMap uvMap_;
    Texture texture_;
};  // CompositeTexture
}  // volcart
}  // texture
