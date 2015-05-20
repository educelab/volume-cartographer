
#include "orderedPCDMesher.h"

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
int width, height;

int orderedPCDMesher(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string outFile){
  std::cout << "Creating mesh from points..." << std::endl;
  std::vector< std::vector< pcl::PointXYZRGB > > VoV;

  // Keep the w+h of our ordered pcd
  width = cloud->width;
  height = cloud->height;

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
  
  write_mesh(outFile);
  return EXIT_SUCCESS;
}

void write_mesh(std::string name) {
  std::ofstream meshFile;
  meshFile.open(name);
  std::cout << "Writing mesh to file..." << std::endl;

  // write header
  meshFile << "ply" << std::endl
           << "format ascii 1.0" << std::endl
           << "comment Created by particle simulation https://github.com/viscenter/registration-toolkit" << std::endl
           << "element dimensions 1" << std::endl
           << "property float width" << std::endl
           << "property float height" << std::endl
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
  
  // write dimensions
  meshFile << width << " " << height << std::endl;

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
