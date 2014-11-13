#include <iostream>

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>

#include "volumepkg.h"

// behavior defines
#define LOADFILES 3
#define CHARGE 0.01
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

int scale;
// NICK: Not sure why we preloaded numslices before. We can count the args
int numslices = 0;
int iteration = 0;
int fieldsize;

std::set<std::string> field_slices;
std::set<std::string>::iterator slice_iterator;
std::set<int> slices_loaded;
std::set<int> slices_seen;

int THRESHOLD = 1;
int realIterations = 0;

//Use this function to produce an ordered PCD where height is the number of iterations
//and width is the size of the particle chain. Invalid locations should be (-1,0,0)
void write_ordered_pcd(std::vector<std::vector<pcl::PointXYZRGB>> storage)
{
  //We usually start at 3, but for the mesher's sanity, assume we start at 1
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.height = realIterations;
  cloud.width = particle_chain.size();
  cloud.points.resize(cloud.height * cloud.width);
  for (int i = 0; i < cloud.height; ++i)
  {
    for (int j = 0; j < cloud.width; ++j)
    {
      cloud.points[j+(i*cloud.width)] = storage[i][j];
    }
  }
  pcl::io::savePCDFileASCII("orderedTest.pcd", cloud);
}


int main(int argc, char* argv[]) {
  if (argc < 5) {
    std::cerr << "Usage:" << std::endl;
    std::cerr << argv[0] << " --quality [1-10] [Path.txt] volpkgpath" << std::endl;
    return (1);  }

  // get scale value from command line
  pcl::console::parse_argument (argc, argv, "--quality", scale);
  if (!((scale>=1)&&(scale<=10))) {
    std::cerr << "ERROR: Incorrect/missing quality value!" << std::endl;
    return (1);
  }

  // read particle chain landmarks
  std::ifstream landmarks_file;
  landmarks_file.open(argv[3]);
  if (landmarks_file.fail()) {
    std::cout << "Path text file could not be opened" << std::endl;
    exit(EXIT_FAILURE);
  }
  while (!landmarks_file.eof()) {
	// REVISIT - Chao 20141104 - new path file format
    double index, a, b;
    landmarks_file >> index >> a >> b;
    chain_landmark.push_back(Particle(index, a, b));
  }
  landmarks_file.close();

  std::string path = argv[4];
  VolumePkg volpkg(path);

  // use landmarks to create chain of particles
  particle_chain.push_back(chain_landmark[0]);
  double total_delta = 0;
  for (int i = 1; i < chain_landmark.size(); ++i) {
	// REVISIT - Chao 20141104 - turned interpolation off here, dense path is generated in pathGen
/*
    Force quarter = (chain_landmark[i] - chain_landmark[i-1]) / 4;
    total_delta += sqrt(quarter.dot(quarter));

    particle_chain.push_back(chain_landmark[i-1] + quarter);
    particle_chain.push_back(chain_landmark[i-1] + quarter*2);
    particle_chain.push_back(chain_landmark[i-1] + quarter*3);
*/
    Force seg = chain_landmark[ i ] - chain_landmark[ i - 1 ];
    total_delta += sqrt( seg.dot( seg ) );

    particle_chain.push_back(chain_landmark[i]);
  }
  spring_resting_x = total_delta / chain_landmark.size();

  // save command line arguments for iteration
  for (int i = 2; i < volpkg.getNumberOfSlices() - 2; ++i) {
    ++numslices;
    field_slices.insert(volpkg.getNormalAtIndex(i));
    // get slice maximum dimension
    if (i == 2) {
      pcl::PointXYZRGBNormal tempmin, tempmax;
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr tempcloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
      pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (volpkg.getNormalAtIndex(i), *tempcloud);
      pcl::getMinMax3D (*tempcloud, tempmin, tempmax);
      if (tempmax.y > tempmax.z) {fieldsize = tempmax.y + 10;} else {fieldsize = tempmax.z + 10;}
    }
  }

  // estimate number of iterations needed
  // TO-DO: Simulation should stop when particles reach the bottom.
  iteration = numslices * scale * 2;
  slice_iterator = field_slices.begin();

  // allocate and initalize force/color fields
  field = new Force**[numslices];
  color = new Color**[numslices];
  for (int i = 0; i < numslices; ++i) {
    field[i] = NULL;
    color[i] = NULL;
  }

  // add some slices to start the simulation
  for (int i = 0; i < 4; ++i) {
    add_slices();
  }

  //particle_status is a bool representing if a particle has passed through the last slice
  std::vector<bool> particle_status;
  for (int i = 0; i < particle_chain.size(); ++i) {
    particle_status.push_back(false);
  }
  //status for the entire chain, this is true until they're all done
  bool particles_status = true;
  //debug value
  int iteration_count = 0;
  //vector of number of stalled iterations per particle. Fixed to 25000 iterations for now.
  std::vector<int> particle_stall_count;
  for (int i = 0; i < particle_chain.size(); ++i) {
    particle_stall_count.push_back(0);
  }
  //This is how we will maintain our ordered structure, vectors of particle chains for each real iteration
  //This has the size of realIterations x particle_chain.size. We assume all slices are loaded
  std::vector<std::vector<pcl::PointXYZRGB>> VoV;
  
  //TODO: Configurable distance threshold
  realIterations = int(numslices/THRESHOLD);
  for (int i = 0; i < realIterations; ++i)
  {
    std::vector<pcl::PointXYZRGB> temp;
    //Invalid particles have x of -1
    for (int j = 0; j <particle_chain.size(); ++j)
    {
      pcl::PointXYZRGB point;
      point.x = -1;
      temp.push_back(point);
    }
    VoV.push_back(temp);
  }
  
  // run particle simulation
  while (particles_status)
  {
    iteration_count++;
    for (int i = 0; i < particle_chain.size(); ++i)
    {
      if (particle_chain[i](0) >= numslices || particle_status[i])
      {
        particle_status[i] = true;
        continue;
      }
      if (particle_stall_count[i] > 25000)
      {
        particle_status[i] = true;
      }
      //TODO: What do we define as a stall? How many stalls?
      particle_stall_count[i] += 1;
      
      uint32_t intensity = (Color)interpolate_intensity(particle_chain[i]);
      uint32_t color = intensity | intensity << 8 | intensity << 16;
      
      //see if a particle has been placed in this slice yet at this chain index
      if (VoV[int(floor(particle_chain[i](0))) - 1][i].x == -1)
      {
        pcl::PointXYZRGB point;
        point.x = particle_chain[i](0);
        point.y = particle_chain[i](1);
        point.z = particle_chain[i](2);
        point.rgb = *reinterpret_cast<float*>(&color);
        VoV[int(floor(particle_chain[i](0))) - 1][i] = point;
      }
    }
    update_particles();
    update_field();
    
    //If all particles are not done, continue
    for (int i = 0; i < particle_chain.size(); i++)
    {
      if (!particle_status[i])
      {
        break;
      }
      if ((i == (particle_chain.size() - 1)) && particle_status[i])
      {
        particles_status = false;
      }
    }
  }
  write_ordered_pcd(VoV);

  exit(EXIT_SUCCESS);
}

// called once for every timestep
void update_particles() {
  slices_seen.clear();
  // for(int i = 0; i < particle_chain.size(); ++i)
  //   particle_chain[i] += intensity_field(particle_chain[i]);

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
  if (*slices_loaded.rbegin() == *slices_seen.rbegin() && *slices_loaded.rbegin() <= numslices) {
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
        x = point->x; // you monster
        y = point->y;
        z = point->z;

        if (field[x] == NULL) {
          slices_loaded.insert(x);
          last_x = x;
          field[x] = new Force*[fieldsize];
          color[x] = new Color*[fieldsize];
          for (int j = 0; j < fieldsize; ++j) {
            field[x][j] = new Force[fieldsize];
            color[x][j] = new Color[fieldsize];
            for (int k = 0; k < fieldsize; ++k) {
              field[x][j][k] = Force(0,0,0);
              color[x][j][k] = 0;
            }
          }
        }

        field[x][y][z](0) = point->normal[0];
        field[x][y][z](1) = point->normal[1];
        field[x][y][z](2) = point->normal[2];
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

  //flip the normal vector to simulate gravity
  Force gravity = Force(1,0,0);
  vector = gravity - (gravity.dot(vector)) / (vector.dot(vector)) * vector;

  cv::normalize(vector);
  vector /= scale;

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

  if (color[x_min] == NULL ||
      color[x_max] == NULL)
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
