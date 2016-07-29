//
// Created by Seth Parker on 7/29/16.
//
/*
 * Purpose: Run volcart::texturing::clothModelingUV() and write results to file for each stage.
 *          Saved file will be read in by the clothTest.cpp file under v-c/testing/texturing.
 */

#include <iostream>

#include "vc_defines.h"
#include "shapes.h"
#include "clothModelingUV.h"
#include "io/objWriter.h"

int main( int argc, char* argv[] ) {

    // Create the mesh writer
    volcart::io::objWriter mesh_writer;

    // Create the test object
    volcart::shapes::Arch arch;

    // Get pinned points for unfurling step
    volcart::texturing::clothModelingUV::PinIDs unfurl;
    unfurl.push_back(0);
    unfurl.push_back(90);

    // Get pinned points for expansion step
    volcart::texturing::clothModelingUV::PinIDs expand;
//    The arch currently doesn't need to be expanded, but leaving this here in case the expansion forces get balanced.
//    expand.push_back(0);
//    expand.push_back(9);
//    expand.push_back(90);
//    expand.push_back(99);

    uint16_t unfurlIt    =  15000;
    uint16_t collisionIt =   5000;
    uint16_t expansionIt =   5000;

    // Run the simulation
    volcart::texturing::clothModelingUV clothUV( arch.itkMesh(), unfurlIt, collisionIt, expansionIt, unfurl, expand);
    clothUV.setAcceleration( volcart::texturing::clothModelingUV::Stage::Unfurl,     10);
    clothUV.setAcceleration( volcart::texturing::clothModelingUV::Stage::Collision, -10);
    clothUV.setAcceleration( volcart::texturing::clothModelingUV::Stage::Expansion, -10);

    clothUV.unfurl();
    mesh_writer.setMesh( clothUV.getMesh() );
    mesh_writer.setPath( "clothUV_Arch_Unfurl.obj" );
    mesh_writer.write();

    clothUV.collide();
    mesh_writer.setMesh( clothUV.getMesh() );
    mesh_writer.setPath( "clothUV_Arch_Collide.obj" );
    mesh_writer.write();

    clothUV.expand();
    mesh_writer.setMesh( clothUV.getMesh() );
    mesh_writer.setPath( "clothUV_Arch_Final.obj" );
    mesh_writer.write();

    return 0;
}