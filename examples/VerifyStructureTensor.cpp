#include <iostream>
#include <tuple>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "volumepkg.h"

using EigenValues = cv::Vec3d;
using EigenVectors = cv::Matx33d;

const StructureTensor zeroST = StructureTensor(0,0,0,0,0,0,0,0,0);

std::pair<EigenValues, EigenVectors> getEigenValuesVectors(StructureTensor t);
int32_t getMaxEigenValueIndex(const EigenValues val);

int main(int argc, char** argv)
{
    if (argc < 5) {
        std::cerr << "Usage:\n";
        std::cerr << "    " << argv[0] << " [volpkg] [x] [y] [z]\n";
        std::exit(1);
    }

    auto path = std::string(argv[1]);
    std::cout << path << std::endl;
    int32_t x = std::stoi(std::string(argv[2]));
    int32_t y = std::stoi(std::string(argv[3]));
    int32_t z = std::stoi(std::string(argv[4]));

    VolumePkg v(path);
    volcart::Volume vol = v.volume();
    assert(z >= 2 && z < v.getNumberOfSlices() - 2 && "z index must not be first or last two points\n");

    auto pathString = vol.getNormalPathAtIndex(z).string();
    auto cloud = new pcl::PointCloud<pcl::PointXYZRGBNormal>();
    auto ret = pcl::io::loadPCDFile(pathString, *cloud);
    if (ret == -1) {
        std::cerr << "couldn't load " << pathString << "\n";
        std::exit(1);
    }

    // Old algo
    // Deal with ZXY --> XYZ bullshit
    int32_t cols = v.getSliceWidth();
    auto pt = cloud->points[y * cols + x];
    std::cout << "\nOld Algo\n";
    std::cout << "estimated normal at index " << pt.y << ", " << pt.z << ", " << pt.x << "\n";
    double correctedNormal[3] = { pt.normal[1], pt.normal[2], pt.normal[0] };
    std::cout << "[" << correctedNormal[0] << ", " << correctedNormal[1] << "," << correctedNormal[2] << "]\n";

    // New algo
    // Print out eigenvector corresponding to max eigenvalue
    std::cout << "\nNew Algo\n";
    auto st = vol.getStructureTensor(x, y, z);
    if (st == zeroST) {
        std::cout << "Structure Tensor was zero\n";
        std::exit(0);
    }
    std::cout << "structure tensor\n" << st << "\n";
    auto eig = getEigenValuesVectors(st);
    std::cout << "eigenvalues\n" << eig.first << "\n" << "eigenvectors\n" << eig.second << "\n";
    int32_t maxIdx = getMaxEigenValueIndex(eig.first);
    std::cout << "Max Eigenvalue index: " << maxIdx << "\n";
    std::cout << "Corresponding eigenvector\n" << eig.second.row(maxIdx) << "\n";
}

std::pair<EigenValues, EigenVectors> getEigenValuesVectors(const StructureTensor t)
{
    EigenValues val;
    EigenVectors vec;
    cv::eigen(t, val, vec);
    return std::make_pair(val, vec);
}

int32_t getMaxEigenValueIndex(const EigenValues val)
{
    std::vector<double> vals = { val(0), val(1), val(2) };
    for (double& v : vals) {
        v = std::fabs(v);
    }
    auto it = std::max_element(vals.begin(), vals.end());
    return it - vals.begin();
}
