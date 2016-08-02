//
// Created by Seth Parker on 6/9/16.
//

#ifndef VC_RENDERING_H
#define VC_RENDERING_H

#include <boost/filesystem.hpp>

#include "../vc_defines.h"
#include "Metadata.h"
#include "Texture.h"

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

#endif //VC_RENDERING_H
