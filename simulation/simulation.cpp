#include <iostream>

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#define FIELDSIZE 400
#define ITERATION 200

typedef cv::Vec3f Particle;
typedef Particle Force;

Force*** field;

// based on the interpolation formula from
// http://paulbourke.net/miscellaneous/interpolation/
Force interpolate_force(Particle point) {
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

int main(int argc, char* argv[]) {

  if (argc == 1) {
    std::cout << "FEED MEEE" << std::endl;
    exit(EXIT_FAILURE);
  }

  // allocate and initalize force field
  field = new Force**[FIELDSIZE];
  for (int i = 0; i < FIELDSIZE; ++i) {
    field[i] = new Force*[FIELDSIZE];
    for (int j = 0; j < FIELDSIZE; ++j) {
      field[i][j] = new Force[FIELDSIZE];
      for (int k = 0; k < FIELDSIZE; ++k) {
        field[i][j][k] = Force(0,0,0);
      }
    }
  }

  // read points from files and add to force field
  int point_counter = 0;
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  for (int i = 1; i < argc; ++i) {
    if (i && i % 10 == 0) {
      std::cout << i << " files read" << std::endl;
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (argv[i], *cloud) == -1) {
      PCL_ERROR ("Couldn't read file\n");
      exit(EXIT_FAILURE);
    } else {
      pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator point;

      for (point = cloud->begin(); point != cloud->end(); ++point) {
        int x, y, z;
        x = point->x;
        y = point->y;
        z = point->z;

        field[x][y][z](0) = point->normal[0];
        field[x][y][z](1) = point->normal[1];
        field[x][y][z](2) = point->normal[2];

        point_counter++;

      }
    }
  }

  // particles can be started anywhere
  // z=2 is the first index with nonzero
  // force field vectors in my test data
  std::vector<Particle> particle;
  for (int i = 0; i < FIELDSIZE - 1; ++i) {
    for (int j = 0; j < FIELDSIZE - 1; ++j) {
      Particle p (i, j, 2);
      Force force = interpolate_force(p);

      if (force.dot(force) != 0) {
        particle.push_back(p);
      }

    }
  }

  std::cout << std::endl << "running particle simulation" << std::endl;

  // run particle simulation
  for (int step = 0; step < ITERATION; ++step) {

    if (step && step % 10 == 0) {
      std::cout << "step " << step << " completed" << std::endl;
    }

    std::ofstream csv;
    csv.open((std::string)"particle.csv." + std::to_string(step));
    csv << "x,y,z" <<std::endl;

    for (int i = 0; i < particle.size(); ++i) {
      csv << particle[i](0) << "," << particle[i](1) << "," << particle[i](2) << std::endl;
      particle[i] += interpolate_force(particle[i]);
    }

    csv.close();
  }

  exit(EXIT_SUCCESS);
}
