#define BOOST_TEST_MODULE OBJReaderTest

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <opencv2/core.hpp>

#include "vc/core/io/OBJReader.hpp"
#include "vc/core/types/Exceptions.hpp"

using namespace volcart;

BOOST_AUTO_TEST_CASE(Read_v_vt_vn_OBJ)
{
    io::OBJReader reader;
    reader.setPath("TexturedPlane.obj");
    BOOST_CHECK_NO_THROW(reader.read());

    auto mesh = reader.getMesh();
    auto uv = reader.getUVMap();
    auto img = reader.getTextureMat();
    BOOST_CHECK_EQUAL(mesh->GetNumberOfPoints(), 16);
    BOOST_CHECK_EQUAL(mesh->GetNumberOfCells(), 18);
    BOOST_CHECK(!uv.empty());
    BOOST_CHECK(!img.empty());
};

BOOST_AUTO_TEST_CASE(Read_v_vt_OBJ)
{
    io::OBJReader reader;
    reader.setPath("TexturedNoNormal.obj");
    BOOST_CHECK_NO_THROW(reader.read());

    auto mesh = reader.getMesh();
    auto uv = reader.getUVMap();
    auto img = reader.getTextureMat();
    BOOST_CHECK_EQUAL(mesh->GetNumberOfPoints(), 16);
    BOOST_CHECK_EQUAL(mesh->GetNumberOfCells(), 18);
    BOOST_CHECK(!uv.empty());
    BOOST_CHECK(!img.empty());
};

BOOST_AUTO_TEST_CASE(Read_v_vn_OBJ)
{
    io::OBJReader reader;
    reader.setPath("UntexturedPlane.obj");
    BOOST_CHECK_NO_THROW(reader.read());

    auto mesh = reader.getMesh();
    auto uv = reader.getUVMap();
    BOOST_CHECK_EQUAL(mesh->GetNumberOfPoints(), 16);
    BOOST_CHECK_EQUAL(mesh->GetNumberOfCells(), 18);
    BOOST_CHECK(uv.empty());
};

BOOST_AUTO_TEST_CASE(Read_v_OBJ)
{
    io::OBJReader reader;
    reader.setPath("Vertices.obj");
    BOOST_CHECK_NO_THROW(reader.read());

    auto mesh = reader.getMesh();
    BOOST_CHECK_EQUAL(mesh->GetNumberOfPoints(), 16);
    BOOST_CHECK_EQUAL(mesh->GetNumberOfCells(), 0);
};

BOOST_AUTO_TEST_CASE(ReadBadOBJ)
{
    io::OBJReader reader;
    reader.setPath("OnlyFaces.obj");
    BOOST_CHECK_THROW(reader.read(), IOException);
};