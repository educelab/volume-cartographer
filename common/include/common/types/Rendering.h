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
        volcart::Metadata metadata();
        std::string id();

        ///// Access Functions /////
        void setTexture( volcart::Texture texture );
        volcart::Texture getTexture();

        void setMesh( VC_MeshType::Pointer mesh );
        VC_MeshType::Pointer getMesh();

    private:
        volcart::Metadata    _metadata;

        volcart::Texture     _texture;
        VC_MeshType::Pointer _mesh;
    };
}
