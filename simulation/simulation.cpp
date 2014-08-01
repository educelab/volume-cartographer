#include <iostream>

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// behavior defines
#define FIELDSIZE 400
#define NUMSLICES 400
#define LOADFILES 3
#define ITERATION 3000
#define CHARGE 0.01
#define SCALE 10
#define SPRING_CONSTANT_K -0.5

typedef uchar Color;
typedef cv::Vec3f Particle;
typedef Particle Force;

typedef struct {
  double x, y, z, nx, ny, nz, s, t;
  uint r, g, b, face_count;
} Vertex;

typedef struct {
  uint v1, v2, v3;
} Face;

void add_vertex(pcl::PointXYZRGB);
void add_face(int, int, int);
void update_normal(int, double, double, double);
void write_mesh();

// forces and particle management
void update_particles();
void update_field();
void add_slices();
Force interpolate_field(Particle);
Force intensity_field(Particle);
Force spring_force(int);

// support functions
double interpolate_intensity(Particle);

// look but don't touch
Force*** field;
Color*** color;
std::vector<Particle> particle_chain;
std::vector<Particle> chain_landmark;
double spring_resting_x;
std::vector<Vertex> vertices;
std::vector<Face> faces;

std::set<std::string> field_slices;
std::set<std::string>::iterator slice_iterator;
std::set<int> slices_loaded;
std::set<int> slices_seen;

int main(int argc, char* argv[]) {
  if (argc == 1) {
    std::cout << "FEED MEEE" << std::endl;
    exit(EXIT_FAILURE);
  }

  std::ifstream landmarks_file;
  landmarks_file.open("landmarks.txt");
  if (landmarks_file.fail()) {
    std::cout << "landmarks.txt could not be opened" << std::endl;
    exit(EXIT_FAILURE);
  }
  while (!landmarks_file.eof()) {
    double a, b;
    landmarks_file >> a;
    landmarks_file.get();
    landmarks_file >> b;
    chain_landmark.push_back(Particle(3, a, b));
  }
  landmarks_file.close();

  // use landmarks to create chain of particles
  particle_chain.push_back(chain_landmark[0]);
  double total_delta = 0;
  for (int i = 1; i < chain_landmark.size(); ++i) {
    Force quarter = (chain_landmark[i] - chain_landmark[i-1]) / 4;
    total_delta += sqrt(quarter.dot(quarter));

    particle_chain.push_back(chain_landmark[i-1] + quarter);
    particle_chain.push_back(chain_landmark[i-1] + quarter*2);
    particle_chain.push_back(chain_landmark[i-1] + quarter*3);
    particle_chain.push_back(chain_landmark[i]);
  }
  spring_resting_x = total_delta / chain_landmark.size();

  // allocate and initalize force/color fields
  field = new Force**[NUMSLICES];
  color = new Color**[NUMSLICES];
  for (int i = 0; i < NUMSLICES; ++i) {
    field[i] = NULL;
    color[i] = NULL;
  }

  for (int i = 1; i < argc; ++i) {
    field_slices.insert((std::string)argv[i]);
  }

  slice_iterator = field_slices.begin();

  for (int i = 0; i < 3; ++i) {
    add_slices();
  }

  pcl::PointCloud<pcl::PointXYZRGB> page;
  for (int step = 0; step < ITERATION; ++step) {
    for (int i = 0; i < particle_chain.size(); ++i) {
      uint32_t intensity = (Color)interpolate_intensity(particle_chain[i]);
      uint32_t color = intensity | intensity << 8 | intensity << 16;

      pcl::PointXYZRGB point;
      point.x = particle_chain[i](0);
      point.y = particle_chain[i](1);
      point.z = particle_chain[i](2);
      point.rgb = *reinterpret_cast<float*>(&color);
      page.push_back(point);

      // build mesh
      add_vertex(point);
      if (step > 0) {
        if (i > 0) {
          int v1, v2, v3, v4, chain_length;
          chain_length = particle_chain.size();
          v1 = step * chain_length + i;
          v2 = v1 - 1;
          v3 = v2 - chain_length;
          v4 = v1 - chain_length;
          add_face(v1, v2, v3);
          add_face(v1, v3, v4);
        }
      }
    }
    update_particles();
    update_field();
  }

  write_mesh();

  pcl::io::savePCDFileASCII("page.pcd", page);
  std::cout << "done" << std::endl;
  exit(EXIT_SUCCESS);
}

void write_mesh() {
  std::ofstream meshFile;
  meshFile.open("mesh.ply");
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

// called once for every timestep
void update_particles() {
  std::vector<Force> workspace(particle_chain.size());
  slices_seen.clear();
  for(int i = 0; i < particle_chain.size(); ++i)
    particle_chain[i] += intensity_field(particle_chain[i]);

  for(int i = 0; i < particle_chain.size(); ++i)
    particle_chain[i] += spring_force(i);

  for(int i = 0; i < particle_chain.size(); ++i)
    particle_chain[i] += interpolate_field(particle_chain[i]);

  for(int i = 0; i < particle_chain.size(); ++i)
    particle_chain[i] += spring_force(i);
}

void update_field() {
  int first_seen = *slices_seen.begin();
  std::vector<int> to_erase;
  for (std::set<int>::iterator it = slices_loaded.begin(); it != slices_loaded.end(); ++it) {
    if (*it < first_seen) {
      to_erase.push_back(*it);
      for (int i = 0; i < FIELDSIZE; ++i) {
        delete field[*it][i];
        delete color[*it][i];
      }
      delete field[*it];
      delete color[*it];
      field[*it] = NULL;
      color[*it] = NULL;
    }
  }
  for (int i = 0; i < to_erase.size(); ++i) {
    std::cout << "deleting slice " << to_erase[i] << std::endl;
    slices_loaded.erase(to_erase[i]);
  }
  if (*slices_loaded.rbegin() == *slices_seen.rbegin()) {
    add_slices();
  }
}

void add_slices() {
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  int last_x = 0;
  for (int i = 0; i < LOADFILES && slice_iterator != field_slices.end(); ++i , ++slice_iterator) {
    std::cout << "loading " << *slice_iterator << std::endl;
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (*slice_iterator, *cloud) == -1) {
      PCL_ERROR ("couldn't read file\n");
      exit(EXIT_FAILURE);
    }

    else {
      pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator point;
      for (point = cloud->begin(); point != cloud->end(); ++point) {
        int x, y, z;
        x = point->z; // you monster
        y = point->y;
        z = point->x;

        if (field[x] == NULL) {
          slices_loaded.insert(x);
          last_x = x;
          field[x] = new Force*[FIELDSIZE];
          color[x] = new Color*[FIELDSIZE];
          for (int j = 0; j < FIELDSIZE; ++j) {
            field[x][j] = new Force[FIELDSIZE];
            color[x][j] = new Color[FIELDSIZE];
            for (int k = 0; k < FIELDSIZE; ++k) {
              field[x][j][k] = Force(0,0,0);
              color[x][j][k] = 0;
            }
          }
        }

        field[x][y][z](0) = point->normal[2]/SCALE;
        field[x][y][z](1) = point->normal[0]/SCALE;
        field[x][y][z](2) = point->normal[1]/SCALE;

        color[x][y][z] = (uchar)(*reinterpret_cast<uint32_t*>(&point->rgb) & 0x0000ff);
      }
    }
  }
}


// based on the interpolation formula from
// http://paulbourke.net/miscellaneous/interpolation/
Force interpolate_field(Particle point) {
  double dx, dy, dz, int_part;
  dx = modf(point(0), &int_part);
  dy = modf(point(1), &int_part);
  dz = modf(point(2), &int_part);

  int x_min, x_max, y_min, y_max, z_min, z_max;
  x_min = (int)point(0);
  x_max = x_min + 1;
  y_min = (int)point(1);
  y_max = y_min + 1;
  z_min = (int)point(2);
  z_max = z_min + 1;

  slices_seen.insert(x_min);
  slices_seen.insert(x_max);

  if (field[x_max] == NULL ||
      field[x_max][y_max] == NULL)
    return Force(0,0,0);

  Force vector =
    field[x_min][y_min][z_min] * (1 - dx) * (1 - dy) * (1 - dz) +
    field[x_max][y_min][z_min] * dx       * (1 - dy) * (1 - dz) +
    field[x_min][y_max][z_min] * (1 - dx) * dy       * (1 - dz) +
    field[x_min][y_min][z_max] * (1 - dx) * (1 - dy) * dz +
    field[x_max][y_min][z_max] * dx       * (1 - dy) * dz +
    field[x_min][y_max][z_max] * (1 - dx) * dy       * dz +
    field[x_max][y_max][z_min] * dx       * dy       * (1 - dz) +
    field[x_max][y_max][z_max] * dx       * dy       * dz;

  return vector;
}

// artificially atract particles to high
// intensity areas
Force intensity_field(Particle point) {
  Force f(0,0,0);
  double interp = interpolate_intensity(point);
  int x_min, x_max, y_min, y_max, z_min, z_max;
  x_min = (int)point(0);
  x_max = x_min + 1;
  y_min = (int)point(1);
  y_max = y_min + 1;
  z_min = (int)point(2);
  z_max = z_min + 1;

  Force neighbor[8] = {
    Force(x_min, y_min, z_min),
    Force(x_max, y_min, z_min),
    Force(x_min, y_max, z_min),
    Force(x_min, y_min, z_max),
    Force(x_max, y_max, z_min),
    Force(x_max, y_min, z_max),
    Force(x_min, y_max, z_max),
    Force(x_max, y_max, z_max),
  };

  for (int i = 0; i < 8; ++i) {
    if (interpolate_intensity(neighbor[i]) < interp) {
      neighbor[i] = Force(0,0,0);
    } else {
      f += (neighbor[i] - point);
    }
  }

  // don't let particles move too far
  f(2) = 0;
  if (f.dot(f) > CHARGE)
    normalize(f, f, CHARGE);

  return f;
}

// particles resist getting too close or too far
// from their neighbors
Force spring_force(int index) {
  Force f(0,0,0);
  if (index != particle_chain.size() - 1) {
    Particle to_right = particle_chain[index] - particle_chain[index + 1];
    double length = sqrt(to_right.dot(to_right));
    normalize(to_right, to_right, SPRING_CONSTANT_K * (length - spring_resting_x));
    f += to_right;
  }
  if (index != 0) {
    Particle to_left = particle_chain[index] - particle_chain[index - 1];
    double length = sqrt(to_left.dot(to_left));
    normalize(to_left, to_left, SPRING_CONSTANT_K * (length - spring_resting_x));
    f += to_left;
  }
  return f;
}

// estimate intensity of volume at particle
double interpolate_intensity(Particle point) {
  double dx, dy, dz, int_part;
  dx = modf(point(0), &int_part);
  dy = modf(point(1), &int_part);
  dz = modf(point(2), &int_part);

  int x_min, x_max, y_min, y_max, z_min, z_max;
  x_min = (int)point(0);
  x_max = x_min + 1;
  y_min = (int)point(1);
  y_max = y_min + 1;
  z_min = (int)point(2);
  z_max = z_min + 1;

  if (color[x_max] == NULL ||
      color[x_max][y_max] == NULL)
    return 0;

  double c =
    color[x_min][y_min][z_min] * (1 - dx) * (1 - dy) * (1 - dz) +
    color[x_max][y_min][z_min] * dx       * (1 - dy) * (1 - dz) +
    color[x_min][y_max][z_min] * (1 - dx) * dy       * (1 - dz) +
    color[x_min][y_min][z_max] * (1 - dx) * (1 - dy) * dz +
    color[x_max][y_min][z_max] * dx       * (1 - dy) * dz +
    color[x_min][y_max][z_max] * (1 - dx) * dy       * dz +
    color[x_max][y_max][z_min] * dx       * dy       * (1 - dz) +
    color[x_max][y_max][z_max] * dx       * dy       * dz;

  return c;
}
