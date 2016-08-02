//
// Created by Seth Parker on 6/9/16.
//

#include "Rendering.h"

namespace fs = boost::filesystem;

using namespace volcart;

///// Constructors/Destructors /////
Rendering::Rendering() {
  _metadata.set<std::string>( "type", "rendering" );
  _metadata.set<std::string>( "id", VC_DATE_TIME() );
}

Rendering::~Rendering() { };

///// Metadata /////
volcart::Metadata Rendering::metadata() { return _metadata; };
std::string Rendering::id() { return _metadata.get<std::string>("id"); };


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
