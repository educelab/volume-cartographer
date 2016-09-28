//
// Created by Seth Parker on 6/9/16.
//
#pragma once

#include <boost/filesystem.hpp>

#include "common/vc_defines.h"
#include "common/types/Metadata.h"
#include "common/types/Texture.h"

namespace volcart {

    class Rendering {
    public:
        ///// Constructors/Destructors /////
        Rendering(); // make new
        Rendering( boost::filesystem::path path ); // load from disk
        ~Rendering();

        ///// Metadata /////
        volcart::Metadata metadata() const;
        std::string id() const;

        ///// Access Functions /////
        void setTexture( volcart::Texture texture );
        volcart::Texture getTexture() const;

        void setMesh( MeshType::Pointer mesh );
        MeshType::Pointer getMesh() const;

    private:
        volcart::Metadata    _metadata;

        volcart::Texture     _texture;
        MeshType::Pointer _mesh;
    };
}
