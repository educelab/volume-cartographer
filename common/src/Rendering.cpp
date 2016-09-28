//
// Created by Seth Parker on 6/9/16.
//

#include "common/types/Rendering.h"

namespace fs = boost::filesystem;

using namespace volcart;

///// Constructors/Destructors /////
Rendering::Rendering() {
  _metadata.set<std::string>( "type", "rendering" );
  _metadata.set<std::string>( "id", DATE_TIME() );
}

Rendering::~Rendering() { };

///// Metadata /////
volcart::Metadata Rendering::metadata() const { return _metadata; };
std::string Rendering::id() const { return _metadata.get<std::string>("id"); };


///// Access Functions /////
void Rendering::setTexture( volcart::Texture texture ) {
  _texture = texture;
}

volcart::Texture Rendering::getTexture() const {
  return _texture;
}

void Rendering::setMesh( MeshType::Pointer mesh ) {
  _mesh = mesh;
}

MeshType::Pointer Rendering::getMesh() const {
  return _mesh;
}
