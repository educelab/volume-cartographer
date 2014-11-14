#include <iostream>

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>

#include "volumepkg.h"

// behavior defines
#define LOADFILES 3
#define SPRING_CONSTANT_K -0.5

typedef cv::Vec3f Particle;
typedef Particle Force;

// forces and particle management
void update_particles();
void update_field();
void add_slices();
Force interpolate_field(Particle);
Force spring_force(int);
bool ps_nand(std::vector<bool>);
void write_ordered_pcd(std::vector<std::vector<pcl::PointXYZRGB> >);

// field globals
Force*** field;
int fieldsize;
std::set<std::string> field_slices;
std::set<std::string>::iterator slice_iterator;
std::set<int> slices_loaded;
std::set<int> slices_seen;

// force globals
std::vector<Particle> particle_chain;
double spring_resting_x;
int gravity_scale;

// misc globals
int numslices;
int THRESHOLD = 1;
int realIterations = 0;
uint32_t COLOR = 0x00777777;

int main(int argc, char* argv[]) {
  std::cout << "vc_simulation" << std::endl;
  if (argc < 5) {
    std::cerr << "Usage:" << std::endl;
    std::cerr << argv[0] << " --gravity_scale [1-10] [Path.txt] volpkgpath" << std::endl;
    exit(EXIT_FAILURE);
  }

  // get gravity scale value from command line
  pcl::console::parse_argument (argc, argv, "--gravity_scale", gravity_scale);
  if (gravity_scale < 1 || gravity_scale > 10) {
    std::cerr << "ERROR: Incorrect/missing gravity_scale value!" << std::endl;
    exit(EXIT_FAILURE);
  }

  // read particle chain landmarks
  std::ifstream landmarks_file;
  int lowestIndex;
  landmarks_file.open(argv[3]);
  if (landmarks_file.fail()) {
    std::cout << "Path text file could not be opened" << std::endl;
    exit(EXIT_FAILURE);
  }

  // REVISIT - Chao 20141104 - new path file format
  std::vector<double> indexes;
  while (!landmarks_file.eof()) {
    double index, a, b;
    landmarks_file >> index >> a >> b;
    particle_chain.push_back(Particle(index, a, b));
    indexes.push_back(index);
  }
  landmarks_file.close();

  // figure out which slice to load first
  int min_index = indexes[0];
  for (int i = 0; i < indexes.size(); ++i) {
    if (min_index > indexes[i]) {
      min_index = indexes[i];
    }
  }

  // we lose 4 slices calculating normals
  std::string path = argv[4];
  VolumePkg volpkg(path);
  numslices = volpkg.getNumberOfSlices() - 4;

  // calculate spring resting distance
  double total_delta = 0;
  for (int i = 1; i < particle_chain.size(); ++i) {
    Force seg = particle_chain[ i ] - particle_chain[ i - 1 ];
    total_delta += sqrt( seg.dot( seg ) );
  }
  spring_resting_x = total_delta / (particle_chain.size() - 1);

  // save normal filepaths for iteration
  for (int i = min_index; i < volpkg.getNumberOfSlices() - 2; ++i) {
    field_slices.insert(volpkg.getNormalAtIndex(i));
  }

  // find out how big the slices need to be
  {
    pcl::PointXYZRGBNormal tempmin, tempmax;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr tempcloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (volpkg.getNormalAtIndex(2), *tempcloud);
    pcl::getMinMax3D (*tempcloud, tempmin, tempmax);
    if (tempmax.y > tempmax.z)
      fieldsize = tempmax.y + 10;
    else
      fieldsize = tempmax.z + 10;
  }

  // allocate and initalize force field
  field = new Force**[volpkg.getNumberOfSlices()];
  for (int i = 0; i < volpkg.getNumberOfSlices(); ++i)
    field[i] = NULL;

  // add some slices to start the simulation
  slice_iterator = field_slices.begin();
  for (int i = 0; i < 4; ++i)
    add_slices();

  //particle_status is a bool representing if a particle has passed through the last slice
  std::vector<bool> particle_status;
  for (int i = 0; i < particle_chain.size(); ++i)
    particle_status.push_back(false);

  //vector of number of stalled iterations per particle. Fixed to 25000 iterations for now.
  std::vector<int> particle_stall_count;
  for (int i = 0; i < particle_chain.size(); ++i)
    particle_stall_count.push_back(0);

  //This is how we will maintain our ordered structure, vectors of particle chains for each real iteration
  //This has the size of realIterations x particle_chain.size. We assume all slices are loaded
  std::vector<std::vector<pcl::PointXYZRGB> > VoV;

  //TODO: Configurable distance threshold
  //Invalid particles have x of -1
  realIterations = int(numslices/THRESHOLD);
  for (int i = 0; i < realIterations; ++i) {
    std::vector<pcl::PointXYZRGB> tmp;
    for (int j = 0; j <particle_chain.size(); ++j) {
      pcl::PointXYZRGB point;
      point.x = -1;
      tmp.push_back(point);
    }
    VoV.push_back(tmp);
  }

  // run particle simulation
  while (ps_nand(particle_status)) {
    for (int i = 0; i < particle_chain.size(); ++i) {
      if (particle_chain[i](0) >= numslices || particle_status[i]) {
        particle_status[i] = true;
        continue;
      }
      if (particle_stall_count[i] > 25000) {
        particle_status[i] = true;
      }

      //TODO: What do we define as a stall? How many stalls?
      particle_stall_count[i] += 1;

      //see if a particle has been placed in this slice yet at this chain index
      if (VoV[int(floor(particle_chain[i](0))) - 1][i].x == -1) {
        pcl::PointXYZRGB point;
        point.x = particle_chain[i](0);
        point.y = particle_chain[i](1);
        point.z = particle_chain[i](2);
        point.rgb = *reinterpret_cast<float*>(&COLOR);
        VoV[int(floor(particle_chain[i](0))) - 1][i] = point;
      }
    }
    update_particles();
    update_field();
  }
  printf("Writing point cloud to file...\n");
  write_ordered_pcd(VoV);
  printf("Segmentation complete!\n");
  exit(EXIT_SUCCESS);
}

// called once for every timestep
void update_particles() {
  slices_seen.clear();

  for(int i = 0; i < particle_chain.size(); ++i)
    particle_chain[i] += spring_force(i);

  for(int i = 0; i < particle_chain.size(); ++i)
    particle_chain[i] += interpolate_field(particle_chain[i]);

  for(int i = 0; i < particle_chain.size(); ++i)
    particle_chain[i] += spring_force(i);
}

// remove/add slices as needed
void update_field() {
  int first_seen = *slices_seen.begin();
  std::vector<int> to_erase;
  for (std::set<int>::iterator it = slices_loaded.begin(); it != slices_loaded.end(); ++it) {
    if (*it < first_seen) {
      to_erase.push_back(*it);
      for (int i = 0; i < fieldsize; ++i) {
        delete field[*it][i];
      }
      delete field[*it];
      field[*it] = NULL;
    }
  }
  for (int i = 0; i < to_erase.size(); ++i) {
    std::cout << "deleting slice " << to_erase[i] << std::endl;
    slices_loaded.erase(to_erase[i]);
  }
  if (*slices_loaded.rbegin() == *slices_seen.rbegin() && *slices_loaded.rbegin() <= numslices) {
    add_slices();
  }
}

void add_slices() {
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  for (int i = 0; i < LOADFILES && slice_iterator != field_slices.end(); ++i , ++slice_iterator) {
    std::cout << "loading " << *slice_iterator << std::endl;

    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (*slice_iterator, *cloud) == -1) {
      PCL_ERROR ("couldn't read file\n");
      exit(EXIT_FAILURE);
    } else {
      pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator point;
      for (point = cloud->begin(); point != cloud->end(); ++point) {
        int x, y, z;
        x = point->x;
        y = point->y;
        z = point->z;

        if (field[x] == NULL) {
          slices_loaded.insert(x);
          field[x] = new Force*[fieldsize];
          for (int j = 0; j < fieldsize; ++j) {
            field[x][j] = new Force[fieldsize];
            for (int k = 0; k < fieldsize; ++k) {
              field[x][j][k] = Force(0,0,0);
            }
          }
        }

        field[x][y][z](0) = point->normal[0];
        field[x][y][z](1) = point->normal[1];
        field[x][y][z](2) = point->normal[2];
      }
    }
  }
}

// interpolation formula from
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

  if (field[x_min] == NULL ||
      field[x_max] == NULL)
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

  Force gravity = Force(1,0,0);
  vector = gravity - (gravity.dot(vector)) / (vector.dot(vector)) * vector;
  cv::normalize(vector);
  vector /= gravity_scale;

  return vector;
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

// nand together particle status
bool ps_nand(std::vector<bool> status) {
  bool result = true;
  for (int i = 0; i < particle_chain.size(); ++i)
    result &= status[i];
  return !result;
}

//Use this function to produce an ordered PCD where height is the number of iterations
//and width is the size of the particle chain. Invalid locations should be (-1,0,0)
//We usually start at 3, but for the mesher's sanity, assume we start at 1
void write_ordered_pcd(std::vector<std::vector<pcl::PointXYZRGB> > storage) {
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.height = realIterations;
  cloud.width = particle_chain.size();
  cloud.points.resize(cloud.height * cloud.width);
  for (int i = 0; i < cloud.height; ++i) {
    for (int j = 0; j < cloud.width; ++j) {
      cloud.points[j+(i*cloud.width)] = storage[i][j];
    }
  }
  pcl::io::savePCDFileASCII("segmented_cloud.pcd", cloud);
}
