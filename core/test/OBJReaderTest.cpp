#include <iostream>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include "vc/core/io/OBJReader.hpp"
#include "vc/core/types/Exceptions.hpp"

using namespace volcart;

class OBJReader : public ::testing::Test
{
public:
    OBJReader() = default;
    io::OBJReader reader;
    std::string path{"vc_core_OBJReader_"};
};

TEST_F(OBJReader, TexturedWithNormals)
{
    reader.setPath(path + "TexturedWithNormals.obj");
    ASSERT_NO_THROW(reader.read());

    auto mesh = reader.getMesh();
    auto uv = reader.getUVMap();
    auto img = reader.getTextureMat();
    EXPECT_EQ(mesh->GetNumberOfPoints(), 16);
    EXPECT_EQ(mesh->GetNumberOfCells(), 18);
    EXPECT_FALSE(uv->empty());
    EXPECT_FALSE(img.empty());
}

TEST_F(OBJReader, Textured)
{
    reader.setPath(path + "Textured.obj");
    ASSERT_NO_THROW(reader.read());

    auto mesh = reader.getMesh();
    auto uv = reader.getUVMap();
    auto img = reader.getTextureMat();
    EXPECT_EQ(mesh->GetNumberOfPoints(), 16);
    EXPECT_EQ(mesh->GetNumberOfCells(), 18);
    EXPECT_FALSE(uv->empty());
    EXPECT_FALSE(img.empty());
}

TEST_F(OBJReader, UntexturedWithNormals)
{
    reader.setPath(path + "UntexturedWithNormals.obj");
    ASSERT_NO_THROW(reader.read());

    auto mesh = reader.getMesh();
    auto uv = reader.getUVMap();
    EXPECT_EQ(mesh->GetNumberOfPoints(), 16);
    EXPECT_EQ(mesh->GetNumberOfCells(), 18);
    EXPECT_TRUE(uv->empty());
}

TEST_F(OBJReader, VariableVertexInfo)
{
    reader.setPath(path + "VariableVertInfo.obj");
    ASSERT_NO_THROW(reader.read());

    auto mesh = reader.getMesh();
    auto uv = reader.getUVMap();
    auto img = reader.getTextureMat();
    EXPECT_EQ(mesh->GetNumberOfPoints(), 16);
    EXPECT_EQ(mesh->GetNumberOfCells(), 18);
    EXPECT_FALSE(uv->empty());
    EXPECT_FALSE(img.empty());
}

TEST_F(OBJReader, PointCloud)
{
    reader.setPath(path + "PointCloud.obj");
    ASSERT_NO_THROW(reader.read());

    auto mesh = reader.getMesh();
    EXPECT_EQ(mesh->GetNumberOfPoints(), 16);
    EXPECT_EQ(mesh->GetNumberOfCells(), 0);
}

TEST_F(OBJReader, Invalid)
{
    reader.setPath(path + "Invalid.obj");
    EXPECT_THROW(reader.read(), IOException);
}