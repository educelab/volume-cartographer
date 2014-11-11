#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>

typedef struct {
  double x, y, z, nx, ny, nz, s, t;
  uint r, g, b, face_count;
} Vertex;

typedef struct {
  uint v1, v2, v3;
} Face;

// mesh generation
void add_vertex(pcl::PointXYZRGB);
void add_face(int, int, int);
void update_normal(int, double, double, double);
void write_mesh(std::string);

std::vector<Vertex> vertices;
std::vector<Face> faces;

int main(int argc, char* argv[]) {
  if (argc < 2)
  {
    std::cerr << "Usage:" << std::endl;
    std::cerr << argv[0] << " orderedFile.pcd" << std::endl;
    return (1);
  }
  
  std::vector<std::vector<pcl::PointXYZRGB>> VoV;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file \n");
    return (-1);
  }

  for (int i = 0; i < cloud->height; ++i)
  {
    std::vector<pcl::PointXYZRGB> current;
    for (int j = 0; j < cloud->width; ++j)
    {
      current.push_back(cloud->points[j+(i*cloud->width)]);
      add_vertex(cloud->points[j+(i*cloud->width)]);
    }
    VoV.push_back(current);
  }
  //width is particle chain size, height is number of iterations
  for (int i = 0; i < cloud->height; ++i)
  {
    for (int j = 0; j < cloud->width; ++j)
    {
      if (i > 0 && j > 0)
      {
        if ( (VoV[i][j].x <= 0) || (VoV[i][j-1].x <= 0) || (VoV[i-1][j].x <= 0) || (VoV[i-1][j-1].x <= 0) )
        {
//          std::cout << "bad vertex in face" << std:: endl;
          continue;
        }
        int v1, v2, v3, v4, chain_length;
        chain_length = cloud->width;
        v1 = i * chain_length + j;
        v2 = v1 - 1;
        v3 = v2 - chain_length;
        v4 = v1 - chain_length;
        add_face(v1, v2, v3);
        add_face(v1, v3, v4);
      }
    }
  }
  std::string outfile = argv[1];
  //replace the extension
  int dot = outfile.find_last_of(".");
  outfile = outfile.substr(0,dot) + ".pcl";
  
  write_mesh(outfile);
  exit(EXIT_SUCCESS);
}


void write_mesh(std::string name) {
  std::ofstream meshFile;
  meshFile.open(name);
  std::cout << "creating mesh file" << std::endl;

  // write header
  meshFile << "ply" << std::endl
           << "format ascii 1.0" << std::endl
           << "comment Created by particle simulation https://github.com/viscenter/registration-toolkit" << std::endl
           << "element vertex " << vertices.size() << std::endl
           << "property float x" << std::endl
           << "property float y" << std::endl
           << "property float z" << std::endl
           << "property float nx" << std::endl
           << "property float ny" << std::endl
           << "property float nz" << std::endl
           << "property float s" << std::endl
           << "property float t" << std::endl
           << "property uchar red" << std::endl
           << "property uchar green" << std::endl
           << "property uchar blue" << std::endl
           << "element face " << faces.size() << std::endl
           << "property list uchar int vertex_indices" << std::endl
           << "end_header" << std::endl;

  // write vertex information
  for (int i = 0; i < vertices.size(); i++) {
    Vertex v = vertices[i];
    meshFile << v.x << " "
             << v.y << " "
             << v.z << " "
             << v.nx << " "
             << v.ny << " "
             << v.nz << " "
             << v.s << " "
             << v.t << " "
             << v.r << " "
             << v.g << " "
             << v.b << std::endl;
  }

  // write face information
  for (int i = 0; i < faces.size(); i++) {
    Face f = faces[i];
    meshFile << "3 " << f.v1 << " " << f.v2 << " " << f.v3 << std::endl;
  }

  meshFile.close();
}

void add_face(int v1, int v2, int v3) {
  Face f;
  f.v1 = v1;
  f.v2 = v2;
  f.v3 = v3;
  faces.push_back(f);

  // calculate vertex normals (average of surface normals of each triangle)

  // get surface normal of this triangle
  // variable names from http://math.stackexchange.com/questions/305642
  double nx, ny, nz, vx, vy, vz, wx, wy, wz, magnitude;

  Vertex vt1 = vertices[v1];
  Vertex vt2 = vertices[v2];
  Vertex vt3 = vertices[v3];

  vx = vt2.x - vt1.x;
  vy = vt2.y - vt1.y;
  vz = vt2.z - vt1.z;

  wx = vt3.x - vt1.x;
  wy = vt3.y - vt1.y;
  wz = vt3.z - vt1.z;

  nx = (vy * wz) - (vz * wy);
  ny = (vz * wx) - (vx * wz);
  nz = (vx * wy) - (vy * wx);

  // normalize
  magnitude = sqrt(nx*nx + ny*ny + nz*nz);
  nx /= magnitude;
  ny /= magnitude;
  nz /= magnitude;

  // update the vertex normals
  update_normal(v1, nx, ny, nz);
  update_normal(v2, nx, ny, nz);
  update_normal(v3, nx, ny, nz);
}

void update_normal(int vertex, double nx_in, double ny_in, double nz_in) {
  // recalculate average (unaverage, add new component, recalculate average)
  Vertex v = vertices[vertex];
  v.nx = (v.nx * v.face_count + nx_in) / (v.face_count + 1);
  v.ny = (v.ny * v.face_count + ny_in) / (v.face_count + 1);
  v.nz = (v.nz * v.face_count + nz_in) / (v.face_count + 1);
  v.face_count++;
  vertices[vertex] = v;
}

void add_vertex(pcl::PointXYZRGB point) {
  Vertex v;
  v.x = point.x;
  v.y = point.y;
  v.z = point.z;
  v.nx = 0;
  v.ny = 0;
  v.nz = 0;
  v.s = 0;
  v.t = 0;
  v.r = point.r;
  v.g = point.g;
  v.b = point.b;
  v.face_count = 0;
  vertices.push_back(v);
}
