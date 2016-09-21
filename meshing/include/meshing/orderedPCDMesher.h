#pragma once

#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#include <boost/filesystem/path.hpp>

namespace volcart {
namespace meshing {

struct Vertex {
  double x, y, z, nx, ny, nz, s, t;
  uint r, g, b, face_count;
};

struct Face {
  uint v1, v2, v3;
};

class Mesh {
  public:
  // mesh generation
    void add_vertex(pcl::PointXYZRGB);
    void add_face(int, int, int);
    void update_normal(int, double, double, double);
    void write_mesh(boost::filesystem::path path);
    void setDimensions( int lWidth, int lHeight ) { width = lWidth; height = lHeight; };
  private:
    int width, height;
    std::vector<Vertex> vertices;
    std::vector<Face> faces;
};

int orderedPCDMesher(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, boost::filesystem::path outFile);

}// namespace meshing
}// namespace volcart
