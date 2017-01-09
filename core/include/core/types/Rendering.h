//
// Created by Seth Parker on 6/9/16.
//
#pragma once

#include <boost/filesystem.hpp>

#include "core/types/Metadata.h"
#include "core/types/Texture.h"
#include "core/vc_defines.h"

namespace volcart
{

class Rendering
{
public:
    ///// Constructors/Destructors /////
    Rendering();                              // make new
    Rendering(boost::filesystem::path path);  // load from disk

    ///// Metadata /////
    volcart::Metadata metadata() const;
    std::string id() const;

    ///// Access Functions /////
    void setTexture(volcart::Texture texture);
    volcart::Texture getTexture() const;

    void setMesh(ITKMesh::Pointer mesh);
    ITKMesh::Pointer getMesh() const;

private:
    volcart::Metadata _metadata;

    volcart::Texture _texture;
    ITKMesh::Pointer _mesh;
};
}
