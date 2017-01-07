//
// Created by Seth Parker on 6/9/16.
//
#pragma once

#include <boost/filesystem/path.hpp>

#include "core/types/Metadata.h"
#include "core/types/Texture.h"
#include "core/vc_defines.h"

namespace volcart
{

class Rendering
{
public:
    ///// Constructors/Destructors /////
    Rendering();
    explicit Rendering(boost::filesystem::path path);

    ///// Metadata /////
    volcart::Metadata metadata() const { return _metadata; }
    std::string id() const { return _metadata.get<std::string>("id"); }

    ///// Access Functions /////
    void setTexture(volcart::Texture texture) { _texture = std::move(texture); }
    volcart::Texture getTexture() const { return _texture; }

    void setMesh(const ITKMesh::Pointer& mesh) { _mesh = mesh; }
    ITKMesh::Pointer getMesh() const { return _mesh; }

private:
    volcart::Metadata _metadata;
    volcart::Texture _texture;
    ITKMesh::Pointer _mesh;
};
}  // namespace volcart
