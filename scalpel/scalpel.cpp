#include <iostream>
#include <cmath>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "volumepkg.h"

// behavior defnines
#define DEPTH CV_64F
#define PIXEL_DEPTH CV_64FC3

// clarity defines
#define DIMENSION 3
#define X_COMPONENT 0
#define Y_COMPONENT 1
#define Z_COMPONENT 2

// clarity typedefs
typedef cv::Matx<double,DIMENSION,DIMENSION> StructureTensor;
typedef std::vector<std::vector<StructureTensor> > TensorGrid;
typedef std::deque<TensorGrid> TensorDeque;
typedef std::deque<cv::Mat> MatDeque;
typedef cv::Vec3d Gradient;
typedef cv::Matx<double,DIMENSION,DIMENSION> EigenVectors;
typedef cv::Vec3d EigenValues;
typedef pcl::PointCloud<pcl::PointXYZRGBNormal> Cloud;

// clarity functions
cv::Mat gradient_3d(MatDeque);
TensorGrid tensorize(cv::Mat);
Cloud process_tensors(TensorDeque, int);
StructureTensor gradient_to_tensor(Gradient);
std::pair<int,double> scan_eigenvalues(EigenValues);

int main(int argc, char* argv[]) {
  if (argc != 2) {
    std::cout << "Usage: " << argv[0] << "volpkgpath" << std::endl;
    exit(EXIT_FAILURE);
  }

  std::string path = argv[1];
  VolumePkg volpkg(path);

  int num_slices = volpkg.getNumberOfSlices();
  int num_characters = 0;
  while (num_slices > 0) {
    num_characters += 1;
    num_slices /= 10;
  }

  MatDeque volume;
  TensorDeque field;
  for(int i = 0; i < volpkg.getNumberOfSlices(); ++i) {
    volume.push_back(volpkg.getSliceAtIndex(i));

    // we need three ct slices to calculate the gradient
    if (volume.size() == 3) {
      field.push_back( tensorize( gradient_3d(volume) ) );
      volume.pop_front();
    }

    // average tensors with neighbors and save normals to disk
    if (field.size() == 3) {
      int index = i - 2;

      std::stringstream outfile;
      outfile << std::setw(num_characters) << std::setfill('0') << index << ".pcd";
      
      pcl::io::savePCDFileASCII(path + "surface_normals/"+ outfile.str(), process_tensors(field, index));
      field.pop_front();
    }
  }

  exit(EXIT_SUCCESS);
}

cv::Mat gradient_3d(MatDeque raw) {
  cv::Mat left_image = raw[0];
  cv::Mat center_image = raw[1];
  cv::Mat right_image = raw[2];

  cv::Mat xyz_gradient = cv::Mat::zeros(left_image.size(), PIXEL_DEPTH);

  // these two are easy
  cv::Mat x_gradient;
  cv::Mat y_gradient;
  Scharr(center_image, x_gradient, DEPTH, 1, 0);
  Scharr(center_image, y_gradient, DEPTH, 0, 1);

  // we need three ct slices to calculate this component
  cv::Mat z_gradient = cv::Mat(xyz_gradient.size(), DEPTH);
  for (int i = 0; i < xyz_gradient.rows; ++i) {
    cv::Mat z_slice  = cv::Mat(3, center_image.cols, DEPTH);

    left_image.row(i).copyTo(z_slice.row(0));
    center_image.row(i).copyTo(z_slice.row(1));
    right_image.row(i).copyTo(z_slice.row(2));

    Scharr(z_slice, z_slice, DEPTH, 0, 1);
    z_slice.row(1).copyTo(z_gradient.row(i));
  }

  // combine gradients into single image
  for (int i = 0; i < xyz_gradient.rows; ++i) {
    for (int j = 0; j < xyz_gradient.cols; ++j) {
      Gradient* d = xyz_gradient.ptr<Gradient>(i, j);
      (*d)(0) = x_gradient.at<double>(i, j);
      (*d)(1)= y_gradient.at<double>(i, j);
      (*d)(2) = z_gradient.at<double>(i, j);
    }
  }

  return xyz_gradient;
}

// maps gradient_to_tensor over \/
TensorGrid tensorize(cv::Mat gradient) {
  TensorGrid tg(gradient.rows);
  for (int i = 0; i < gradient.rows; ++i) {
    for (int j = 0; j < gradient.cols; ++j) {
      Gradient *g;
      g = gradient.ptr<Gradient>(i,j);
      tg[i].push_back(gradient_to_tensor(*g));
    }
  }
  return tg;
}

// get normal vectors for middle slice in the deque
Cloud process_tensors(TensorDeque field, int index) {
  Cloud cloud;
  int rows = field[0].size();
  int cols = field[0][0].size();
  
  TensorGrid average(rows);
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      average[i].push_back(StructureTensor(0,0,0,0,0,0,0,0,0));
    }
  }

  // edges are skipped
  for (int i = 1; i < rows-1; ++i) {
    for (int j = 1; j < cols-1; ++j) {
      average[i][j] =
        field[0][  i  ][  j  ] +
        field[1][  i  ][  j  ] +
        field[2][  i  ][  j  ] +
        field[1][i + 1][  j  ] +
        field[1][i - 1][  j  ] +
        field[1][  i  ][j + 1] +
        field[1][  i  ][j - 1];
      average[i][j] = (1.0/7.0) * average[i][j];
    }
  }

  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      EigenValues eigenvalues;
      EigenVectors eigenvectors;
      eigen(average[i][j], eigenvalues, eigenvectors);

      std::pair<int,double> eigen_data = scan_eigenvalues(eigenvalues);

      cv::Matx<double, 1, DIMENSION> normal_vector = eigenvectors.row(eigen_data.first);
      pcl::PointXYZRGBNormal point;

      // texturing is going to happen later so the rgb field
      // will be for the difference of the two highest eigenvalues
      point.rgb = eigen_data.second;

      // point.xyz <=> image.index/col/row
      point.x = index;
      point.y = j;
      point.z = i;
      point.normal[0] = normal_vector(Z_COMPONENT);
      point.normal[1] = normal_vector(X_COMPONENT);
      point.normal[2] = normal_vector(Y_COMPONENT);

      cloud.push_back(point);
    }
  }

  return cloud;
}

// create a structure tensor out of a gradient vector
StructureTensor gradient_to_tensor(Gradient g) {
  double Ix = g(0);
  double Iy = g(1);
  double Iz = g(2);
  return StructureTensor
    (Ix*Ix, Ix*Iy, Ix*Iz,
     Ix*Iy, Iy*Iy, Iy*Iz,
     Ix*Iz, Iy*Iz, Iz*Iz);
}

// find and chomp maximum eigenvalue
std::pair<int,double> scan_eigenvalues(EigenValues e) {
  std::pair<int,double> result;

  e(0) = std::abs(e(0));
  e(1) = std::abs(e(1));
  e(2) = std::abs(e(2));

  double max = std::max(e(0), std::max(e(1), e(2)));
  if (max == e(0)) { result.first = 0; }
  if (max == e(1)) { result.first = 1; }
  if (max == e(2)) { result.first = 2; }

  double a = std::abs(e(0) - e(1));
  double b = std::abs(e(1) - e(2));
  double c = std::abs(e(2) - e(0));
  result.second = std::max(a, std::max(b, c));

  return result;
}

