#include "vc/core/util/Debug.hpp"

#include <iomanip>

void volcart::debug::PrintPointCloud(volcart::OrderedPointSet<cv::Vec3d> points, std::string label, bool withCoordinates) {
    // Print point cloud info to console / standard out.
    // Assumes X, Y, Z coordinates for the points with a common Z value per row.
    std::cout << "=== Start of Point Cloud ";
    if (label.length() > 0) {
        std::cout << "(" << label << ") ";
    }
    std::cout << "===" << std::endl;

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

void volcart::debug::PrintPointTable(std::vector<std::vector<cv::Vec3d>> points, std::string label, bool withCoordinates) {
    // Print point table (sectioned into rows) info to console / standard out.
    // Assumes X, Y, Z coordinates for the points with a common Z value per row.
    std::cout << "=== Start of Point Table ";
    if (label.length() > 0) {
        std::cout << "(" << label << ") ";
    }
    std::cout << "===" << std::endl;

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
