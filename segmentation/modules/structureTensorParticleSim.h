#ifndef TENSORPARTICLE_H
#define TENSORPARTICLE_H

#include <iostream>

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>

#include "volumepkg.h"

typedef cv::Vec3f Particle;
typedef Particle Force;

pcl::PointCloud<pcl::PointXYZRGB> structureTensorParticleSim(std::string landmark_path, VolumePkg volpkg, int gravity_scale = 2, int threshold = 1, int endSlice = -1);

#endif