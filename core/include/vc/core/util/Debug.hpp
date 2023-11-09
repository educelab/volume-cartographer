#pragma once

/** @file */

#include "vc/core/types/OrderedPointSet.hpp"

namespace volcart
{
    void PrintPointCloud(volcart::OrderedPointSet<cv::Vec3d> points, bool withCoordinates = false) {
        // Print point cloud info to console / standard out.
        // Assumes X, Y, Z coordinates for the points with a common Z value per row.
        std::cout << "=== Start of Point Cloud ===" << std::endl;

        for (int i = 0; i < points.height(); i++) {
            auto row = points.getRow(i);
            std::cout << "Row " << std::setfill('0') << std::setw(4) << i << " | ";
            std::cout << "Start Index: " << std::setfill('0') << std::setw(4) << i * points.width();
            std::cout  << " Z: "  << std::setfill('0') << std::setw(4) << (int)row[0][2] << " | ";
            if (withCoordinates) {
                for(auto point : row) {
                    std::cout << "(" << std::fixed << std::setprecision(3) << std::setfill('0') << std::setw(4) << point[0] << ", " << point[1] << ") | ";
                }
            }
            std::cout << std::endl;
        }
    }

    void PrintPointTable(std::vector<std::vector<cv::Vec3d>> points, bool withCoordinates = false) {
        // Print point table (sectioned into rows) info to console / standard out.
        // Assumes X, Y, Z coordinates for the points with a common Z value per row.
        std::cout << "=== Start of Point Table ===" << std::endl;

        int i = 0;
        for (auto row : points) {
            std::cout << "Row " << std::setfill('0') << std::setw(4) << i << " | ";
            std::cout << "Start Index: " << std::setfill('0') << std::setw(4) << i * row.size();
            std::cout  << " Z: "  << std::setfill('0') << std::setw(4) << (int)row[0][2] << " | ";
            if (withCoordinates) {
                for (auto point : row) {
                    std::cout << "(" << std::fixed << std::setprecision(3) << std::setfill('0') << std::setw(4) << point[0] << ", " << point[1] << ") | ";
                }
            }
            std::cout << std::endl;
            i++;
        }
    }
};