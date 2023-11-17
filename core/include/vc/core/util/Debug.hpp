#pragma once

/** @file */

#include "vc/core/types/OrderedPointSet.hpp"

namespace volcart::debug
{

    void PrintPointCloud(OrderedPointSet<cv::Vec3d> points, std::string label = "",  bool withCoordinates = false);

    void PrintPointTable(std::vector<std::vector<cv::Vec3d>> points, std::string label = "",  bool withCoordinates = false);

}
