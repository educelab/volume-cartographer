#pragma once

#include <iostream>

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>

#include "volumepkg/volumepkg.h"

#include "segmentation/stps/particle.h"
#include "segmentation/stps/chain.h"

namespace volcart {
    namespace segmentation {
        volcart::OrderedPointSet<volcart::Point3d> structureTensorParticleSim(volcart::OrderedPointSet<volcart::Point3d> segPath, const VolumePkg& volpkg, double gravity_scale = 0.5, int threshold = 1, int endOffset = -1);
    }// namespace segmentation
} // namespace volcart
