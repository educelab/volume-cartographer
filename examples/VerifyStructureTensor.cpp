#include <iostream>
#include <tuple>
#include <array>
#include <cstdio>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "volumepkg.h"

using EigenVector = cv::Vec3d;

EigenVector oldAlgoNormalAtIndex(
    const pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud, const int32_t width,
    const int32_t x, const int32_t y);
pcl::PointCloud<pcl::PointXYZRGBNormal> cloudFromSlice(VolumePkg& v,
                                                       const int32_t z);

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "Usage:\n";
        std::cerr << "    " << argv[0] << " volpkg z-idx [-verbose]\n";
        std::exit(1);
    }

    auto path = std::string(argv[1]);
    int32_t arg_z = std::stoi(std::string(argv[2]));
    bool verbose = false;
    if (argc > 3) {
        auto s = std::string(argv[3]);
        if (s == "-verbose" || s == "-v") {
            verbose = true;
        }
    }

    VolumePkg v(path);
    const volcart::Volume vol = v.volume();
    assert(arg_z >= 2 && arg_z < v.getNumberOfSlices() - 2 &&
           "z index must not be first or last two points\n");

    // Comparison of two eigenvectors for closeness
    auto eigenvectorsAreClose = [](const EigenVector v1, const EigenVector v2,
                                   const double eps) {
        return cv::norm(v2 - v1) < eps;
    };

    // Epsilon for comparison
    const double eps = 0.01;

    // Comparison over whole slice
    auto cloud = cloudFromSlice(v, arg_z);
    int64_t totalCount = 0;
    int64_t sameCount = 0;
    int64_t zeroCount = 0;
    for (int32_t y = 0; y < v.getSliceHeight(); ++y) {
        for (int32_t x = 0; x < v.getSliceWidth(); ++x) {
            // Get old eigenvalue
            auto oldEigvec =
                oldAlgoNormalAtIndex(cloud, v.getSliceWidth(), x, y);

            // Get new eigenvalue
            try {
                auto pairs = vol.eigenPairsAtIndex(x, y, arg_z);
                auto newEigvec = pairs[0].second;
                if (eigenvectorsAreClose(oldEigvec, newEigvec, eps)) {
                    sameCount++;
                } else {
                    if (verbose) {
                        std::cout << "(" << x << ", " << y << ") " << oldEigvec
                                  << ", " << newEigvec << "  "
                                  << cv::norm(oldEigvec - newEigvec) << "\n";
                    }
                }
            } catch (const volcart::ZeroStructureTensorException& ex) {
                zeroCount++;
            }
            totalCount++;
        }
    }

    // Report
    int64_t validCount = totalCount - zeroCount;
    std::printf("zeroCount: %ld (%f%%)\n", zeroCount,
                double(zeroCount) / totalCount);
    std::printf("sameCount: %ld (%f%%)\n", sameCount,
                double(sameCount) / validCount);
    std::printf("diffCount: %ld (%f%%)\n", validCount - sameCount,
                double(validCount - sameCount) / validCount);
}

pcl::PointCloud<pcl::PointXYZRGBNormal> cloudFromSlice(VolumePkg& v,
                                                       const int32_t z)
{
    auto pathString = v.volume().getNormalPathAtIndex(z).string();
    auto cloud = pcl::PointCloud<pcl::PointXYZRGBNormal>();
    if (pcl::io::loadPCDFile(pathString, cloud) == -1) {
        std::cerr << "couldn't load " << pathString << "\n";
        std::exit(1);
    }
    return cloud;
}

EigenVector oldAlgoNormalAtIndex(
    const pcl::PointCloud<pcl::PointXYZRGBNormal>& cloud, const int32_t width,
    const int32_t x, const int32_t y)
{
    // Deal with ZXY --> XYZ bullshit
    auto pt = cloud.points[y * width + x];
    return {pt.normal_y, pt.normal_z, pt.normal_x};
}
