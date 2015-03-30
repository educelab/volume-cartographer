#include <iostream>

#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>

#include "/Users/mike/development/volume-cartographer/volumepkg/volumepkg.h"


/********************************************************************************
SETI FOR SCROLLS

Be prepared to generate about 10GB testing this on
PHerc3-ItalianScan-PhaseContrast-Sample-2013

I didn't feel like messing with randomly sized input images so I made fieldsize
something that worked for PHerc3-ItalianScan-PhaseContrast-Sample-2013.
Threshold should also be looked at. Any "Ideal" value will likely change per
data set.
********************************************************************************/

// Unless you want to generate 80 gazillion files keep threshold high
#define THRESHOLD 1.000e+11

// this should be based on real life/voxel size
#define OUTPUTDIM 64

typedef cv::Vec3f Vector;
typedef struct {
  Vector normal;
  float eigen;
  short intensity;
} Point;
Point*** volume;

int fieldsize = 400;

// global to avoid passing volpkg to every function
int number_of_slices;

// I like to have these up here so main can be closer to the top of the page
short interpolate_intensity(Vector);
void prep(VolumePkg);
void seti();
void slicer(int,int,int,Vector,Vector);

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cout << "no volpkg" << std::endl;
    exit(EXIT_FAILURE);
  }

  std::string volpkgpath(argv[1]);
  VolumePkg volpkg(volpkgpath);
  number_of_slices = volpkg.getNumberOfSlices();

  prep(volpkg);
  seti();

  exit(EXIT_SUCCESS);
}

// cuts an image out of the volume that is perpendicular to the normal at xyz
void slicer(int x, int y, int z, Vector gproj, Vector xyint) {
  cv::Mat m(OUTPUTDIM, OUTPUTDIM, CV_16UC1);
  Vector image_origin = Vector(x,y,z) - (((OUTPUTDIM/2) * gproj) + ((OUTPUTDIM/2) * xyint));
  for (int imgx = 0; imgx < OUTPUTDIM; ++imgx) {
    for (int imgy = 0; imgy < OUTPUTDIM; ++imgy) {
      m.at<short>(imgx,imgy) = interpolate_intensity(image_origin + (imgx * gproj) + (imgy * xyint));
    }
  }
  std::stringstream ss;
  ss << "seti_" << std::setw(3) << std::setfill('0') << x << "_" << std::setw(3) << std::setfill('0') << y << "_" << std::setw(3) << std::setfill('0') << z << ".png";
  imwrite(ss.str(), m);
}

// cuts out an image for each voxel that is more planar than THRESHOLD
// plane is parameterized as gravity projection from simulation and
// a n intersection with the yz plane
// (which is technically a cross product but it simplifies nicely)
void seti() {
  Vector gravity(1,0,0);
  for (int x = 2; x < number_of_slices - 2; ++x) {
    if (x % 10 == 0) {
      std::cout << "seti scanning at slice " << x << std::endl;
    }
    for (int y = 1; y < fieldsize - 1; ++y) {
      for (int z = 1; z < fieldsize - 1; ++z) {
        Vector current = volume[x][y][z].normal;
        // I would also like to suggest that we not pull out an image if it already exists
        if (volume[x][y][z].eigen < THRESHOLD) {
          continue;
        }

        Vector gproj = gravity - (current.dot(gravity)/current.dot(current)) * current;
        Vector xyint = Vector(0, -current(2), current(1));
        normalize(gproj);
        normalize(xyint);
        slicer(x, y, z, gproj, xyint);
      }
    }
  }
}

// same interpolation from simulation
short interpolate_intensity(Vector point) {
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

  if (x_max <= 0 || x_min >= number_of_slices)
    return 0;
  if (y_max <= 0 || y_min >= fieldsize)
    return 0;
  if (z_max <= 0 || z_min >= fieldsize)
    return 0;

  if (volume[x_min] == NULL ||
      volume[x_max] == NULL)
    return 0;

  short color =
    volume[x_min][y_min][z_min].intensity * (1 - dx) * (1 - dy) * (1 - dz) +
    volume[x_max][y_min][z_min].intensity * dx       * (1 - dy) * (1 - dz) +
    volume[x_min][y_max][z_min].intensity * (1 - dx) * dy       * (1 - dz) +
    volume[x_min][y_min][z_max].intensity * (1 - dx) * (1 - dy) * dz +
    volume[x_max][y_min][z_max].intensity * dx       * (1 - dy) * dz +
    volume[x_min][y_max][z_max].intensity * (1 - dx) * dy       * dz +
    volume[x_max][y_max][z_min].intensity * dx       * dy       * (1 - dz) +
    volume[x_max][y_max][z_max].intensity * dx       * dy       * dz;

  return color;
}

// allocate volume to store data
void prep(VolumePkg volpkg) {
  volume = new Point**[number_of_slices];
  for (int i = 0; i < number_of_slices; ++i) {
    volume[i] = new Point*[fieldsize];
    for (int j = 0; j < fieldsize; ++j) {
      volume[i][j] = new Point[fieldsize];
      for (int k = 0; k < fieldsize; ++k) {
        volume[i][j][k].normal = Vector(0,0,0);
        volume[i][j][k].eigen = 0;
        volume[i][j][k].intensity = 0;
      }
    }
  }

  for (int i = 2; i < number_of_slices - 2; ++i) {
    std::string slicename = volpkg.getNormalAtIndex(i);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    std::cout << "loading " << slicename << std::endl;

    // I have a history of not being able to keep rows and cols in the
    // right order so it would be nice if someone would verify that data
    // from  cv::Mat is being put in the right place
    cv::Mat in_orig = volpkg.getSliceAtIndex(i);
    for (int imgx = 0; imgx < in_orig.cols; ++imgx) {
      for (int imgy = 0; imgy < in_orig.rows; ++imgy) {
        volume[i][imgx][imgy].intensity = in_orig.at<short>(imgx,imgy);
      }
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (slicename, *cloud) == -1) {
      PCL_ERROR ("couldn't read file\n");
      exit(EXIT_FAILURE);
    } else {
      pcl::PointCloud<pcl::PointXYZRGBNormal>::iterator point;
      for (point = cloud->begin(); point != cloud->end(); ++point) {
        int x, y, z;
        x = point->x;
        y = point->y;
        z = point->z;

        volume[x][y][z].normal(0) = point->normal[0];
        volume[x][y][z].normal(1) = point->normal[1];
        volume[x][y][z].normal(2) = point->normal[2];

        volume[x][y][z].eigen = point->rgb;
      }
    }
  }
}
