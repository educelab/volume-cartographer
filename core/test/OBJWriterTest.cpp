#include <gtest/gtest.h>

#include <cstddef>

#include "vc/core/io/OBJWriter.hpp"
#include "vc/core/shapes/Plane.hpp"
#include "vc/core/types/SimpleMesh.hpp"
#include "vc/testing/ParsingHelpers.hpp"

namespace vc = volcart;
namespace vcshapes = volcart::shapes;
namespace vctest = volcart::testing;

class OBJWriter : public ::testing::Test
{
public:
    OBJWriter()
    {
        // create a test mesh
        vcshapes::Plane plane;
        mesh = plane.itkMesh();
        writer.setMesh(mesh);
    }

    vc::ITKMesh::Pointer mesh;
    std::string path{"vc_core_OBJWriter_"};
    vc::io::OBJWriter writer;
};

TEST_F(OBJWriter, UntexturedMesh)
{
    // Write test file
    path += "Untextured.obj";
    writer.setPath(path);
    ASSERT_NO_THROW(writer.write());

    // load in written data
    vc::SimpleMesh saved;
    vctest::ParsingHelpers::ParseOBJFile(path, saved.verts, saved.faces);

    // compare number of points and cells for equality
    EXPECT_EQ(mesh->GetNumberOfPoints(), saved.verts.size());
    EXPECT_EQ(mesh->GetNumberOfCells(), saved.faces.size());

    // Check vertex values
    std::size_t idx = 0;
    vc::ITKPixel origN;
    for (const auto& v : saved.verts) {
        // Check 3D Position
        auto orig = mesh->GetPoint(idx);
        EXPECT_DOUBLE_EQ(v.x, orig[0]);
        EXPECT_DOUBLE_EQ(v.y, orig[1]);
        EXPECT_DOUBLE_EQ(v.z, orig[2]);

        // Check Normal
        mesh->GetPointData(idx, &origN);
        EXPECT_DOUBLE_EQ(v.nx, origN[0]);
        EXPECT_DOUBLE_EQ(v.ny, origN[1]);
        EXPECT_DOUBLE_EQ(v.nz, origN[2]);

        idx++;
    }

    // Check face vertex IDs
    idx = 0;
    for (const auto& f : saved.faces) {
        auto orig = mesh->GetCells()->GetElement(idx);

        EXPECT_EQ(f.v1, orig->GetPointIds()[0]);
        EXPECT_EQ(f.v2, orig->GetPointIds()[1]);
        EXPECT_EQ(f.v3, orig->GetPointIds()[2]);

        idx++;
    }
}