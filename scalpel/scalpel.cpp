#include <iostream>
#include <cmath>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// behavior defnines
#define BLUR_SIZE 3
#define THRESHOLD 5.06659e+05
#define DEPTH CV_64F
#define PIXEL_DEPTH CV_64FC3

// clarity defines
#define DIMENSION 3
#define X_COMPONENT 0
#define Y_COMPONENT 1
#define Z_COMPONENT 2

// convenience defines
#define KERNEL cv::Size(BLUR_SIZE,BLUR_SIZE)

// misc defines
#define ARROW_SCALE 1

// typedefs for clarity
typedef cv::Matx<double, DIMENSION, DIMENSION> StructureTensor;
typedef cv::Vec3d Gradient;
typedef cv::Vec3d Pixel;
typedef cv::Matx<double, DIMENSION, DIMENSION> EigenVectors;
typedef cv::Vec3d EigenValues;

// create a structure tensor out of a gradient vector
StructureTensor* gradient_to_tensor(Gradient g) {
  double Ix = g(0);
  double Iy = g(1);
  double Iz = g(2);
  return new StructureTensor
    (Ix*Ix, Ix*Iy, Ix*Iz,
     Ix*Iy, Iy*Iy, Iy*Iz,
     Ix*Iz, Iy*Iz, Iz*Iz);
}

// find and chomp maximum eigenvalue
int scan_eigenvalues(EigenValues e) {
  e(0) = std::abs(e(0));
  e(1) = std::abs(e(1));
  e(2) = std::abs(e(2));
  double a = std::abs(e(0) - e(1));
  double b = std::abs(e(1) - e(2));
  double c = std::abs(e(2) - e(0));
  double m = std::max(a, std::max(b, c));
  double max = std::max(e(0), std::max(e(1), e(2)));
  if  (m < THRESHOLD) { return -1; }
  if      (max == e(0)) { return 0; }
  if      (max == e(1)) { return 1; }
  if      (max == e(2)) { return 2; }
  return -1;
}

// import scroll
// scroll.unwrap()
int main(int argc, char* argv[]) {
  if (argc < 5) {
    std::cout << "Usage: "
              << argv[0]
              << " leftimage centerimage rightimage outputimage [tensor?]"
              << std::endl;
    exit(EXIT_FAILURE);
  }

  // create gradient images
  if (argc == 5) {
    std::cout << "creating gradient image of " << argv[2] << std::endl;

    // load images
    cv::Mat left_image = cv::imread(argv[1]);
    cv::Mat center_image = cv::imread(argv[2]);
    cv::Mat right_image = cv::imread(argv[3]);

    if (!left_image.data) { std::cout << argv[1] << " could not be read" << std::endl; exit(EXIT_FAILURE); }
    if (!center_image.data) { std::cout << argv[2] << " could not be read" << std::endl; exit(EXIT_FAILURE); }
    if (!right_image.data) { std::cout << argv[3] << " could not be read" << std::endl; exit(EXIT_FAILURE); }

    cv::Mat xyz_gradient = cv::Mat::zeros(center_image.size(), PIXEL_DEPTH);

    cvtColor(left_image, left_image, CV_BGR2GRAY);
    cvtColor(center_image, center_image, CV_BGR2GRAY);
    cvtColor(right_image, right_image, CV_BGR2GRAY);

    GaussianBlur(left_image, left_image, KERNEL, 0);
    GaussianBlur(center_image, center_image, KERNEL, 0);
    GaussianBlur(right_image, right_image, KERNEL, 0);

    equalizeHist(left_image, left_image);
    equalizeHist(center_image,center_image);
    equalizeHist(right_image, right_image);

    // calculate xy gradients
    cv::Mat x_gradient, y_gradient;
    Scharr(center_image, x_gradient, DEPTH, 1, 0);
    Scharr(center_image, y_gradient, DEPTH, 0, 1);

    // calculate z gradient
    cv::Mat z_gradient = cv::Mat(xyz_gradient.rows, xyz_gradient.cols, DEPTH);
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

    // write gradient image to disk
    cv::FileStorage fs(argv[4], cv::FileStorage::WRITE);
    fs << "gradient" << xyz_gradient;
  }

  // construct and analyze structure tensors
  else {
    std::cout << "analyzing gradient images to produce " << argv[4] << std::endl;

    cv::Mat left_image, center_image, right_image;
    cv::FileStorage left(argv[1], cv::FileStorage::READ);
    cv::FileStorage cent(argv[2], cv::FileStorage::READ);
    cv::FileStorage righ(argv[3], cv::FileStorage::READ);
    left["gradient"] >> left_image;
    cent["gradient"] >> center_image;
    righ["gradient"] >> right_image;


    // you monster
    StructureTensor**** st_volume = new StructureTensor***[3];
    for (int i = 0; i < 3; ++i) {
      st_volume[i] = new StructureTensor**[center_image.rows];
      for (int j = 0; j < center_image.rows; ++j) {
        st_volume[i][j] = new StructureTensor*[center_image.cols];
      }
    }

    // put structure tensors in a volume for easy access
    for (int i = 0; i < center_image.rows; ++i) {
      for (int j = 0; j < center_image.cols; ++j) {
        Gradient *l, *r, *c;
        l = left_image.ptr<Gradient>(i,j);
        c = center_image.ptr<Gradient>(i,j);
        r = right_image.ptr<Gradient>(i,j);
        st_volume[0][i][j] = gradient_to_tensor(*l);
        st_volume[1][i][j] = gradient_to_tensor(*c);
        st_volume[2][i][j] = gradient_to_tensor(*r);
      }
    }

    // average tensors of interest with their neighbors
    StructureTensor*** average = new StructureTensor**[center_image.rows];
    for (int i = 0; i < center_image.rows; ++i) {
      average[i] = new StructureTensor*[center_image.cols];
      for (int j = 0; j <center_image.cols; j++) {
        average[i][j] = new StructureTensor
          (0, 0, 0,
           0, 0, 0,
           0, 0, 0);
      }
    }
    for (int i = 1; i < center_image.rows-1; ++i) { // edges are skipped
      for (int j = 1; j < center_image.cols-1; ++j) {
        *average[i][j] =
          *st_volume[0][  i  ][  j  ] +
          *st_volume[1][  i  ][  j  ] +
          *st_volume[2][  i  ][  j  ] +
          *st_volume[1][i + 1][  j  ] +
          *st_volume[1][i - 1][  j  ] +
          *st_volume[1][  i  ][j + 1] +
          *st_volume[1][  i  ][j - 1];
        *average[i][j] = (1.0/7.0) * *average[i][j];
      }
    }

    // run analysis on averaged tensors
    cv::Mat arrow = cv::Mat::zeros(center_image.size(), PIXEL_DEPTH);
    pcl::PointCloud<pcl::PointXYZRGBNormal> cloud;

    for (int i = 0; i < center_image.rows; ++i) {
      for (int j = 0; j < center_image.cols; ++j) {
        int index;
        EigenValues eigen_values;
        EigenVectors eigen_vectors;

        eigen(*average[i][j], eigen_values, eigen_vectors);
        index = scan_eigenvalues(eigen_values);

        if (index != -1) {
          cv::Matx<double, 1, DIMENSION> normal_vector = eigen_vectors.row(index);
          cv::Matx<double, 1, DIMENSION> gravity;
          gravity(X_COMPONENT) = 0;
          gravity(Y_COMPONENT) = 0;
          gravity(Z_COMPONENT) = 1;

          // project gravity onto the plane defined by each normal
          normal_vector = gravity - (gravity.dot(normal_vector)) / (normal_vector.dot(normal_vector)) * normal_vector;


          cv::Point arrow_offset(normal_vector(X_COMPONENT) * ARROW_SCALE,
                                 normal_vector(Y_COMPONENT) * ARROW_SCALE);

          double vector_length = sqrt(normal_vector(X_COMPONENT) * normal_vector(X_COMPONENT) +
                                      normal_vector(Y_COMPONENT) * normal_vector(Y_COMPONENT) +
                                      normal_vector(Z_COMPONENT) * normal_vector(Z_COMPONENT));

          cv::Vec3d vector_color(normal_vector(X_COMPONENT) / vector_length,
                                 normal_vector(Y_COMPONENT) / vector_length,
                                 normal_vector(Z_COMPONENT) / vector_length);

          normal_vector = (1/vector_length) * normal_vector;

          vector_color *= 255;
          vector_color(X_COMPONENT) = std::abs(vector_color(X_COMPONENT));
          vector_color(Y_COMPONENT) = std::abs(vector_color(Y_COMPONENT));
          vector_color(Z_COMPONENT) = std::abs(vector_color(Z_COMPONENT));

          line(arrow,
               cv::Point(j, i),
               cv::Point(j, i) + arrow_offset,
               cv::Scalar(vector_color));

          pcl::PointXYZRGBNormal point;
          uint32_t color =
            (uint32_t)vector_color(X_COMPONENT) |
            (uint32_t)vector_color(Y_COMPONENT) << 8 |
            (uint32_t)vector_color(Z_COMPONENT) << 16;
          point.x = i;
          point.y = j;
          point.z = atoi(argv[5]);
          point.rgb = *reinterpret_cast<float*>(&color);
          point.normal[0] = normal_vector(X_COMPONENT);
          point.normal[1] = normal_vector(Y_COMPONENT);
          point.normal[2] = normal_vector(Z_COMPONENT);
          cloud.push_back(point);

        }
      }
    }

    // write images to disk
    pcl::io::savePCDFileASCII((std::string)"cloud"+ argv[5] +".pcd", cloud);
    cv::imwrite(argv[4], arrow);
  }

  exit(EXIT_SUCCESS);
}
