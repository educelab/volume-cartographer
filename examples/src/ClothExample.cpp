//
// Created by Seth Parker on 7/29/16.
//
/*
 * Purpose: Run vce::texturing::ClothModelingUVMapping() and write results
 * to file for each stage.
 *          Saved file will be read in by the clothTest.cpp file under
 * v-c/testing/texturing.
 */

#include <iostream>

#include "vc/core/io/OBJWriter.hpp"
#include "vc/core/shapes/Arch.hpp"
#include "vc/experimental/texturing/ClothModelingUVMapping.hpp"

namespace vce = volcart::experimental;

int main()
{
    // Create the mesh writer
    volcart::io::OBJWriter mesh_writer;

    // Create the test object
    volcart::shapes::Arch arch;

    // Get pinned points for unfurling step
    vce::texturing::ClothModelingUVMapping::VertIDList unfurl;
    unfurl.push_back(0);
    unfurl.push_back(90);

    // Get pinned points for expansion step
    vce::texturing::ClothModelingUVMapping::VertIDList expand;
    //    The arch currently doesn't need to be expanded, but leaving this here
    //    in case the expansion forces get balanced.
    //    expand.push_back(0);
    //    expand.push_back(9);
    //    expand.push_back(90);
    //    expand.push_back(99);

    uint16_t unfurlIt = 20000;
    uint16_t collisionIt = 0;
    uint16_t expansionIt = 5000;

    // Run the simulation
    vce::texturing::ClothModelingUVMapping clothUV(
        arch.itkMesh(), unfurlIt, collisionIt, expansionIt, unfurl, expand);
    clothUV.setAcceleration(
        vce::texturing::ClothModelingUVMapping::Stage::Unfurl, 10);
    clothUV.setAcceleration(
        vce::texturing::ClothModelingUVMapping::Stage::Collision, -10);
    clothUV.setAcceleration(
        vce::texturing::ClothModelingUVMapping::Stage::Expansion, -10);

    clothUV.unfurl();
    mesh_writer.setMesh(clothUV.getMesh());
    mesh_writer.setPath("clothUV_Arch_Unfurl.obj");
    mesh_writer.write();

    clothUV.collide();
    mesh_writer.setMesh(clothUV.getMesh());
    mesh_writer.setPath("clothUV_Arch_Collide.obj");
    mesh_writer.write();

    clothUV.expand();
    mesh_writer.setMesh(clothUV.getMesh());
    mesh_writer.setPath("clothUV_Arch_Final.obj");
    mesh_writer.write();

    return 0;
}
