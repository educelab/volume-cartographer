#include <gtest/gtest.h>

#include "vc/core/shapes/Arch.hpp"
#include "vc/core/types/SimpleMesh.hpp"
#include "vc/experimental/texturing/ClothModelingUVMapping.hpp"
#include "vc/testing/ParsingHelpers.hpp"
#include "vc/testing/TestingUtils.hpp"

using namespace volcart;
namespace vce = volcart::experimental;

/*
 *
 *    FIXTURES
 *
 */

class CreateArchClothUVFixture : public ::testing::Test
{
public:
    CreateArchClothUVFixture()
    {

        // get ITK Mesh
        _in_Mesh = _Arch.itkMesh();

        // Setup params
        _unfurlIt = 20000;
        _collisionIt = 0;
        _expansionIt = 5000;

        _unfurlPins.push_back(0);
        _unfurlPins.push_back(90);

        // Setup the simulation
        vce::texturing::ClothModelingUVMapping clothUV(
            _in_Mesh, _unfurlIt, _collisionIt, _expansionIt, _unfurlPins,
            _expandPins);
        clothUV.setAcceleration(
            vce::texturing::ClothModelingUVMapping::Stage::Unfurl, 10);
        clothUV.setAcceleration(
            vce::texturing::ClothModelingUVMapping::Stage::Collision, -10);
        clothUV.setAcceleration(
            vce::texturing::ClothModelingUVMapping::Stage::Expansion, -10);

        // Run the simulation in parts
        clothUV.unfurl();
        _out_Mesh_unfurl = clothUV.getMesh();
        clothUV.collide();
        _out_Mesh_collide = clothUV.getMesh();
        clothUV.expand();
        _out_Mesh_final = clothUV.getMesh();

        // Load pre-generated output from file
        volcart::testing::ParsingHelpers::ParseOBJFile(
            "clothUV_Arch_Unfurl.obj", _SavedPoints_unfurl, _SavedCells_unfurl);
        volcart::testing::ParsingHelpers::ParseOBJFile(
            "clothUV_Arch_Collide.obj", _SavedPoints_collide,
            _SavedCells_collide);
        volcart::testing::ParsingHelpers::ParseOBJFile(
            "clothUV_Arch_Final.obj", _SavedPoints_final, _SavedCells_final);
    }

    // declare Arch mesh
    volcart::shapes::Arch _Arch;
    ITKMesh::Pointer _in_Mesh;
    ITKMesh::Pointer _out_Mesh_unfurl, _out_Mesh_collide, _out_Mesh_final;

    // Simulation
    vce::texturing::ClothModelingUVMapping::VertIDList _unfurlPins;
    vce::texturing::ClothModelingUVMapping::VertIDList _expandPins;
    uint16_t _unfurlIt;
    uint16_t _collisionIt;
    uint16_t _expansionIt;

    std::vector<SimpleMesh::Vertex> _SavedPoints_unfurl;
    std::vector<SimpleMesh::Cell> _SavedCells_unfurl;

    std::vector<SimpleMesh::Vertex> _SavedPoints_collide;
    std::vector<SimpleMesh::Cell> _SavedCells_collide;

    std::vector<SimpleMesh::Vertex> _SavedPoints_final;
    std::vector<SimpleMesh::Cell> _SavedCells_final;
};

/*
 *
 *    TEST CASES
 *
 */

TEST_F(CreateArchClothUVFixture, ArchClothUVTesT)
{

    ///// Unfurl /////
    // check size of uvMap and number of points in mesh
    EXPECT_EQ(
        _out_Mesh_unfurl->GetNumberOfPoints(), _in_Mesh->GetNumberOfPoints());
    EXPECT_EQ(
        _out_Mesh_unfurl->GetNumberOfPoints(), _SavedPoints_unfurl.size());

    // check uvmap against original mesh input pointIDs
    for (size_t point = 0; point < _SavedPoints_unfurl.size(); ++point) {
        volcart::testing::SmallOrClose(
            _out_Mesh_unfurl->GetPoint(point)[0], _SavedPoints_unfurl[point].x,
            1.0, 5.0);
        volcart::testing::SmallOrClose(
            _out_Mesh_unfurl->GetPoint(point)[1], _SavedPoints_unfurl[point].y,
            1.0, 5.0);
        volcart::testing::SmallOrClose(
            _out_Mesh_unfurl->GetPoint(point)[2], _SavedPoints_unfurl[point].z,
            1.0, 5.0);
    }

    ///// Collide /////
    // check size of uvMap and number of points in mesh
    EXPECT_EQ(
        _out_Mesh_collide->GetNumberOfPoints(), _in_Mesh->GetNumberOfPoints());
    EXPECT_EQ(
        _out_Mesh_collide->GetNumberOfPoints(), _SavedPoints_collide.size());

    // check uvmap against original mesh input pointIDs
    for (size_t point = 0; point < _SavedPoints_collide.size(); ++point) {
        volcart::testing::SmallOrClose(
            _out_Mesh_collide->GetPoint(point)[0],
            _SavedPoints_collide[point].x, 1.0, 5.0);
        volcart::testing::SmallOrClose(
            _out_Mesh_collide->GetPoint(point)[1],
            _SavedPoints_collide[point].y, 1.0, 5.0);
        volcart::testing::SmallOrClose(
            _out_Mesh_collide->GetPoint(point)[2],
            _SavedPoints_collide[point].z, 1.0, 5.0);
    }

    ///// Expand/Final /////
    // check size of uvMap and number of points in mesh
    EXPECT_EQ(
        _out_Mesh_final->GetNumberOfPoints(), _in_Mesh->GetNumberOfPoints());
    EXPECT_EQ(_out_Mesh_final->GetNumberOfPoints(), _SavedPoints_final.size());

    // check uvmap against original mesh input pointIDs
    for (size_t point = 0; point < _SavedPoints_final.size(); ++point) {
        volcart::testing::SmallOrClose(
            _out_Mesh_final->GetPoint(point)[0], _SavedPoints_final[point].x,
            1.0, 5.0);
        volcart::testing::SmallOrClose(
            _out_Mesh_final->GetPoint(point)[1], _SavedPoints_final[point].y,
            1.0, 5.0);
        volcart::testing::SmallOrClose(
            _out_Mesh_final->GetPoint(point)[2], _SavedPoints_final[point].z,
            1.0, 5.0);
    }
}