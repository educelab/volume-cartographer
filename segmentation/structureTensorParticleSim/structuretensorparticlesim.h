#pragma once

#ifndef TENSORPARTICLE_H
#define TENSORPARTICLE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class VolumePkg;

namespace volcart
{
namespace segmentation
{
pcl::PointCloud<pcl::PointXYZRGB> structureTensorParticleSim(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr segPath, VolumePkg& volpkg,
    const double gravity_scale = 0.5, const int32_t threshold = 1,
    const int32_t endOffset = -1);
}  // namespace segmentation
}  // namespace volcart

#endif
