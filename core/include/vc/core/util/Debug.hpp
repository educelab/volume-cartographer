#pragma once

/** @file */

#include "vc/core/types/OrderedPointSet.hpp"

namespace volcart
{
    void PrintPointCloud(volcart::OrderedPointSet<cv::Vec3d> points) {
        // Print point cloud info to console / standard out. Assumes X, Y, Z coordinates for the points
        // with a common Z value per row.
        for (int i = 0; i < points.height(); i++) {
            auto row = points.getRow(i);
            std::cout << "Row " << std::setfill('0') << std::setw(4) << i << " | ";
            std::cout << "Start Index: " << std::setfill('0') << std::setw(4) << i * points.width();
            std::cout  << " Z: "  << std::setfill('0') << std::setw(4) << (int)row[0][2] << " | ";
            for(auto point : row) {
                std::cout << "(" << std::fixed << std::setprecision(3) << std::setfill('0') << std::setw(4) << point[0] << ", " << point[1] << ") | ";
            }
            std::cout << std::endl;
        }
    }
};