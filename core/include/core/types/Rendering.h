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
    volcart::Metadata metadata() const { return metadata_; }
    std::string id() const { return metadata_.get<std::string>("id"); }

    ///// Access Functions /////
    void setTexture(volcart::Texture texture) { texture_ = std::move(texture); }
    volcart::Texture getTexture() const { return texture_; }

    void setMesh(const ITKMesh::Pointer& mesh) { mesh_ = mesh; }
    ITKMesh::Pointer getMesh() const { return mesh_; }

private:
    volcart::Metadata metadata_;
    volcart::Texture texture_;
    ITKMesh::Pointer mesh_;
};
}
