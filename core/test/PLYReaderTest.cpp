#define BOOST_TEST_MODULE PLYReader

#include <boost/test/unit_test.hpp>
#include <boost/test/unit_test_log.hpp>

#include "core/io/PLYReader.h"
#include "core/io/plyWriter.h"
#include "core/shapes/Arch.h"
#include "core/shapes/Cone.h"
#include "core/shapes/Plane.h"
#include "core/shapes/Sphere.h"
#include "core/vc_defines.h"
#include "testing/parsingHelpers.h"
#include "testing/testingUtils.h"

struct ReadITKPlaneMeshFixture {
    ReadITKPlaneMeshFixture()
    {
        _in_PlaneMesh = _Plane.itkMesh();

        volcart::io::plyWriter writer("PLYReader_plane.ply", _in_PlaneMesh);
        writer.write();
    }

    volcart::ITKMesh::Pointer _in_PlaneMesh;
    volcart::ITKMesh::Pointer _read_PlaneMesh;
    volcart::shapes::Plane _Plane;
};

struct ReadITKArchMeshFixture {
    ReadITKArchMeshFixture()
    {
        _in_ArchMesh = _Arch.itkMesh();

        volcart::io::plyWriter writer("PLYReader_arch.ply", _in_ArchMesh);
        writer.write();

        volcart::testing::ParsingHelpers::parsePlyFile(
            "PLYReader_arch.ply", _SavedArchPoints, _SavedArchCells);
    }

    volcart::ITKMesh::Pointer _in_ArchMesh;
    volcart::ITKMesh::Pointer _read_ArchMesh;
    volcart::shapes::Arch _Arch;

    std::vector<volcart::Vertex> _SavedArchPoints;
    std::vector<volcart::Cell> _SavedArchCells;
};

struct ReadITKConeMeshFixture {
    ReadITKConeMeshFixture()
    {
        _in_ConeMesh = _Cone.itkMesh();

        volcart::io::plyWriter writer("PLYReader_cone.ply", _in_ConeMesh);
        writer.write();
    }

    volcart::ITKMesh::Pointer _in_ConeMesh;
    volcart::ITKMesh::Pointer _read_ConeMesh;
    volcart::shapes::Cone _Cone;
};

struct ReadITKSphereMeshFixture {
    ReadITKSphereMeshFixture()
    {
        _in_SphereMesh = _Sphere.itkMesh();

        volcart::io::plyWriter writer("PLYReader_sphere.ply", _in_SphereMesh);
        writer.write();
    }

    volcart::ITKMesh::Pointer _in_SphereMesh;
    volcart::ITKMesh::Pointer _read_SphereMesh;
    volcart::shapes::Sphere _Sphere;
};

BOOST_FIXTURE_TEST_CASE(ReadArchMeshTest, ReadITKArchMeshFixture)
{
    volcart::io::PLYReader reader("PLYReader_arch.ply");
    reader.read();
    _read_ArchMesh = reader.getMesh();
    BOOST_CHECK_EQUAL(
        _in_ArchMesh->GetNumberOfPoints(), _read_ArchMesh->GetNumberOfPoints());
    BOOST_CHECK_EQUAL(
        _in_ArchMesh->GetNumberOfCells(), _read_ArchMesh->GetNumberOfCells());
    for (unsigned long pnt_id = 0; pnt_id < _in_ArchMesh->GetNumberOfPoints();
         pnt_id++) {
        volcart::testing::SmallOrClose(
            _in_ArchMesh->GetPoint(pnt_id)[0],
            _read_ArchMesh->GetPoint(pnt_id)[0]);
        volcart::testing::SmallOrClose(
            _in_ArchMesh->GetPoint(pnt_id)[1],
            _read_ArchMesh->GetPoint(pnt_id)[1]);
        volcart::testing::SmallOrClose(
            _in_ArchMesh->GetPoint(pnt_id)[2],
            _read_ArchMesh->GetPoint(pnt_id)[2]);

        volcart::ITKPixel in_normal;
        volcart::ITKPixel read_normal;

        _in_ArchMesh->GetPointData(pnt_id, &in_normal);
        _read_ArchMesh->GetPointData(pnt_id, &read_normal);

        volcart::testing::SmallOrClose(in_normal[0], read_normal[0]);
        volcart::testing::SmallOrClose(in_normal[1], read_normal[1]);
        volcart::testing::SmallOrClose(in_normal[2], read_normal[2]);
    }

    for (unsigned long cell_id = 0; cell_id < _in_ArchMesh->GetNumberOfCells();
         cell_id++) {
        volcart::ITKCell::CellAutoPointer in_C;
        _in_ArchMesh->GetCell(cell_id, in_C);
        volcart::ITKCell::CellAutoPointer read_C;
        _read_ArchMesh->GetCell(cell_id, read_C);
        BOOST_CHECK_EQUAL(in_C->GetPointIds()[0], read_C->GetPointIds()[0]);
        BOOST_CHECK_EQUAL(in_C->GetPointIds()[1], read_C->GetPointIds()[1]);
        BOOST_CHECK_EQUAL(in_C->GetPointIds()[2], read_C->GetPointIds()[2]);
    }
}

BOOST_FIXTURE_TEST_CASE(ReadPlaneMeshTest, ReadITKPlaneMeshFixture)
{
    volcart::io::PLYReader reader("PLYReader_plane.ply");
    reader.read();
    _read_PlaneMesh = reader.getMesh();
    BOOST_CHECK_EQUAL(
        _in_PlaneMesh->GetNumberOfPoints(),
        _read_PlaneMesh->GetNumberOfPoints());
    BOOST_CHECK_EQUAL(
        _in_PlaneMesh->GetNumberOfCells(), _read_PlaneMesh->GetNumberOfCells());
    for (unsigned long pnt_id = 0; pnt_id < _in_PlaneMesh->GetNumberOfPoints();
         pnt_id++) {
        volcart::testing::SmallOrClose(
            _in_PlaneMesh->GetPoint(pnt_id)[0],
            _read_PlaneMesh->GetPoint(pnt_id)[0]);
        volcart::testing::SmallOrClose(
            _in_PlaneMesh->GetPoint(pnt_id)[1],
            _read_PlaneMesh->GetPoint(pnt_id)[1]);
        volcart::testing::SmallOrClose(
            _in_PlaneMesh->GetPoint(pnt_id)[2],
            _read_PlaneMesh->GetPoint(pnt_id)[2]);

        volcart::ITKPixel in_normal;
        volcart::ITKPixel read_normal;

        _in_PlaneMesh->GetPointData(pnt_id, &in_normal);
        _read_PlaneMesh->GetPointData(pnt_id, &read_normal);

        volcart::testing::SmallOrClose(in_normal[0], read_normal[0]);
        volcart::testing::SmallOrClose(in_normal[1], read_normal[1]);
        volcart::testing::SmallOrClose(in_normal[2], read_normal[2]);
    }

    for (unsigned long cell_id = 0; cell_id < _in_PlaneMesh->GetNumberOfCells();
         cell_id++) {
        volcart::ITKCell::CellAutoPointer in_C;
        _in_PlaneMesh->GetCell(cell_id, in_C);
        volcart::ITKCell::CellAutoPointer read_C;
        _read_PlaneMesh->GetCell(cell_id, read_C);
        BOOST_CHECK_EQUAL(in_C->GetPointIds()[0], read_C->GetPointIds()[0]);
        BOOST_CHECK_EQUAL(in_C->GetPointIds()[1], read_C->GetPointIds()[1]);
        BOOST_CHECK_EQUAL(in_C->GetPointIds()[2], read_C->GetPointIds()[2]);
    }
}

BOOST_FIXTURE_TEST_CASE(ReadConeMeshTest, ReadITKConeMeshFixture)
{
    volcart::io::PLYReader reader("PLYReader_cone.ply");
    reader.read();
    _read_ConeMesh = reader.getMesh();
    BOOST_CHECK_EQUAL(
        _in_ConeMesh->GetNumberOfPoints(), _read_ConeMesh->GetNumberOfPoints());
    BOOST_CHECK_EQUAL(
        _in_ConeMesh->GetNumberOfCells(), _read_ConeMesh->GetNumberOfCells());
    for (unsigned long pnt_id = 0; pnt_id < _in_ConeMesh->GetNumberOfPoints();
         pnt_id++) {
        volcart::testing::SmallOrClose(
            _in_ConeMesh->GetPoint(pnt_id)[0],
            _read_ConeMesh->GetPoint(pnt_id)[0]);
        volcart::testing::SmallOrClose(
            _in_ConeMesh->GetPoint(pnt_id)[1],
            _read_ConeMesh->GetPoint(pnt_id)[1]);
        volcart::testing::SmallOrClose(
            _in_ConeMesh->GetPoint(pnt_id)[2],
            _read_ConeMesh->GetPoint(pnt_id)[2]);

        volcart::ITKPixel in_normal;
        volcart::ITKPixel read_normal;

        _in_ConeMesh->GetPointData(pnt_id, &in_normal);
        _read_ConeMesh->GetPointData(pnt_id, &read_normal);

        volcart::testing::SmallOrClose(in_normal[0], read_normal[0]);
        volcart::testing::SmallOrClose(in_normal[1], read_normal[1]);
        volcart::testing::SmallOrClose(in_normal[2], read_normal[2]);
    }

    for (unsigned long cell_id = 0; cell_id < _in_ConeMesh->GetNumberOfCells();
         cell_id++) {
        volcart::ITKCell::CellAutoPointer in_C;
        _in_ConeMesh->GetCell(cell_id, in_C);
        volcart::ITKCell::CellAutoPointer read_C;
        _read_ConeMesh->GetCell(cell_id, read_C);
        BOOST_CHECK_EQUAL(in_C->GetPointIds()[0], read_C->GetPointIds()[0]);
        BOOST_CHECK_EQUAL(in_C->GetPointIds()[1], read_C->GetPointIds()[1]);
        BOOST_CHECK_EQUAL(in_C->GetPointIds()[2], read_C->GetPointIds()[2]);
    }
}

BOOST_FIXTURE_TEST_CASE(ReadSphereMeshTest, ReadITKSphereMeshFixture)
{
    volcart::io::PLYReader reader("PLYReader_sphere.ply");
    reader.read();
    _read_SphereMesh = reader.getMesh();
    BOOST_CHECK_EQUAL(
        _in_SphereMesh->GetNumberOfPoints(),
        _read_SphereMesh->GetNumberOfPoints());
    BOOST_CHECK_EQUAL(
        _in_SphereMesh->GetNumberOfCells(),
        _read_SphereMesh->GetNumberOfCells());
    for (unsigned long pnt_id = 0; pnt_id < _in_SphereMesh->GetNumberOfPoints();
         pnt_id++) {
        volcart::testing::SmallOrClose(
            _in_SphereMesh->GetPoint(pnt_id)[0],
            _read_SphereMesh->GetPoint(pnt_id)[0]);
        volcart::testing::SmallOrClose(
            _in_SphereMesh->GetPoint(pnt_id)[1],
            _read_SphereMesh->GetPoint(pnt_id)[1]);
        volcart::testing::SmallOrClose(
            _in_SphereMesh->GetPoint(pnt_id)[2],
            _read_SphereMesh->GetPoint(pnt_id)[2]);

        volcart::ITKPixel in_normal;
        volcart::ITKPixel read_normal;

        _in_SphereMesh->GetPointData(pnt_id, &in_normal);
        _read_SphereMesh->GetPointData(pnt_id, &read_normal);

        volcart::testing::SmallOrClose(in_normal[0], read_normal[0]);
        volcart::testing::SmallOrClose(in_normal[1], read_normal[1]);
        volcart::testing::SmallOrClose(in_normal[2], read_normal[2]);
    }

    for (unsigned long cell_id = 0;
         cell_id < _in_SphereMesh->GetNumberOfCells(); cell_id++) {
        volcart::ITKCell::CellAutoPointer in_C;
        _in_SphereMesh->GetCell(cell_id, in_C);
        volcart::ITKCell::CellAutoPointer read_C;
        _read_SphereMesh->GetCell(cell_id, read_C);
        BOOST_CHECK_EQUAL(in_C->GetPointIds()[0], read_C->GetPointIds()[0]);
        BOOST_CHECK_EQUAL(in_C->GetPointIds()[1], read_C->GetPointIds()[1]);
        BOOST_CHECK_EQUAL(in_C->GetPointIds()[2], read_C->GetPointIds()[2]);
    }
}
