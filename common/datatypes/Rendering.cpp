//
// Created by Seth Parker on 6/9/16.
//

#include "Rendering.h"

namespace fs = boost::filesystem;

namespace volcart {

    ///// Constructors/Destructors /////
    Rendering::Rendering() {
      _metadata.setValue( "type", "rendering" );
      _metadata.setValue( "id", VC_DATE_TIME() );
    }

    Rendering::~Rendering() { };

    ///// Metadata /////
    volcart::Metadata Rendering::metadata() { return _metadata; };
    std::string Rendering::id() { return _metadata.getString("id"); };


    ///// Access Functions /////
    void Rendering::setTexture( volcart::Texture texture ) {
      _texture = texture;
    }

    volcart::Texture Rendering::getTexture() {
      return _texture;
    }

    void Rendering::setMesh( VC_MeshType::Pointer mesh ) {
      _mesh = mesh;
    }

    VC_MeshType::Pointer Rendering::getMesh() {
      return _mesh;
    }

}