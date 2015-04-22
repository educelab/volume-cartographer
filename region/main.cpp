#include <iostream>
#include <cmath>
#include <queue>

#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>

#include "volumepkg.h"
#include "voxel.h"
#include "region.h"

Voxel**** volume;
int fieldsize = 400;
int number_of_slices;
void prep(VolumePkg);
void start_region();

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cout << "no volpkg" << std::endl;
    exit(EXIT_FAILURE);
  }

  std::string volpkgpath(argv[1]);
  VolumePkg volpkg(volpkgpath);
  number_of_slices = volpkg.getNumberOfSlices();

  prep(volpkg);
  start_region();

  exit(EXIT_SUCCESS);
}

void start_region() {
  std::priority_queue<Voxel> pq;
  for (int i = 0; i < number_of_slices; ++i) {
    for (int j = 0; j < fieldsize; ++j) {
      for (int k = 0; k < fieldsize; ++k) {
        if (volume[i][j][k] != NULL) {
          pq.push(*volume[i][j][k]);}}}}
  
  Voxel master = pq.top();
  pq.pop();
  Vector pos = master.pos();
  int x = pos(0);
  int y = pos(1);
  int z = pos(2);
  Region r(volume[x][y][z]);

  for (int i = 0; i < 4096; ++i) {
    Voxel master = pq.top();
    pq.pop();
    Vector pos = master.pos();
    int x = pos(0);
    int y = pos(1);
    int z = pos(2);
    r.insert(volume[x][y][z]);
  }
  pq = std::priority_queue<Voxel>();

  for (int i = 0; i < 8; ++i) {
    n = r.grow();
  }
  r.write();
}

// allocate volume to store data
void prep(VolumePkg volpkg) {
  volume = new Voxel***[number_of_slices];
  for (int i = 0; i < number_of_slices; ++i) {
    volume[i] = new Voxel**[fieldsize];
    for (int j = 0; j < fieldsize; ++j) {
      volume[i][j] = new Voxel*[fieldsize];
      for (int k = 0; k < fieldsize; ++k) {
        volume[i][j][k] = NULL;
      }
    }
  }

  for (int i = 2; i < number_of_slices - 2; ++i) {
    std::string slicename = volpkg.getNormalAtIndex(i);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    std::cout << "loading " << slicename << std::endl;
    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (slicename, *cloud) == -1) {
      PCL_ERROR ("couldn't read file\n");
      exit(EXIT_FAILURE);
    } else {
      pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator point;
      for (point = cloud->begin(); point != cloud->end(); ++point) {
        int x = point->x;
        int y = point->y;
        int z = point->z;
        volume[x][y][z] = new Voxel (Vector(point->x,
                                            point->y,
                                            point->z),
                                     Vector(point->normal[0],
                                            point->normal[1],
                                            point->normal[2]),
                                     point->rgb);
      }
    }
  }
}
