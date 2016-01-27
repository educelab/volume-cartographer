#include <iostream>
#include <random>
#include <sstream>
#include <tuple>
#include <cmath>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include <H5Cpp.h>
#include "volumepkg.h"

namespace fs = boost::filesystem;

struct CartesianCoord {
    int32_t x, y, z;
    CartesianCoord() = default;
    CartesianCoord(int32_t x, int32_t y, int32_t z) : x(x), y(y), z(z) {}
    operator cv::Point3i() const { return {x, y, z}; }
};
std::ostream& operator<<(std::ostream& s, const CartesianCoord& c)
{
    return s << "[" << c.x << ", " << c.y << ", " << c.z << "]";
}

struct PolarCoord {
    double r, theta;
    PolarCoord() = default;
    PolarCoord(double r, double theta) : r(r), theta(theta) {}
};
std::ostream& operator<<(std::ostream& s, const PolarCoord& p)
{
    return s << "[" << p.r << ", " << p.theta << "]";
}

CartesianCoord polarToCartesian(const PolarCoord coord, const int32_t z);
void draw(const volcart::Volume& vol, const CartesianCoord c);
template <typename DType>
void writeVolumeToH5File(const std::string& filename,
                         const volcart::Tensor3D<DType>& volume);
void writeStructureTensorEigToH5File(const std::string& filename,
                                     const StructureTensor& st,
                                     const EigenPairs& pairs);

// Center of the dataset on XY-plane
int32_t g_centerX;
int32_t g_centerY;

int main(int argc, char** argv)
{
    if (argc < 9) {
        std::cerr << "Usage:\n"
                  << "    " << argv[0]
                  << " volumepkg N outer-r inner-r centerX centerY zmin zmax "
                     "output-dir\n";
        std::exit(1);
    }
    auto volpkg = VolumePkg(std::string(argv[1]));
    const int32_t nsamples = std::stoi(std::string(argv[2]));
    const int32_t outerRadius = std::stoi(std::string(argv[3]));
    const int32_t innerRadius = std::stoi(std::string(argv[4]));
    g_centerX = std::stoi(std::string(argv[5]));
    g_centerY = std::stoi(std::string(argv[6]));
    const int32_t zmin = std::stoi(std::string(argv[7]));
    const int32_t zmax = std::stoi(std::string(argv[8]));
    // const auto outputDir = fs::path(argv[9]);

    const volcart::Volume vol = volpkg.volume();

    // Make generator and distributions
    std::random_device rd;
    std::default_random_engine e(rd());
    std::uniform_real_distribution<double> rDist(innerRadius, outerRadius);
    std::uniform_real_distribution<double> thetaDist(0, 2 * M_PI);
    std::uniform_int_distribution<int32_t> zDist(zmin, zmax + 1);

    // Start generating
    CartesianCoord c;
    while (true) {
        double theta = thetaDist(e);
        double r = rDist(e);
        int32_t z = zDist(e);
        std::cout << "r: " << r << ", theta: " << theta << ", z: " << z
                  << std::endl;
        c = polarToCartesian({r, theta}, z);

        // Volume bounds checking
        if (c.x < 0 || c.x > volpkg.getSliceWidth() || c.y < 0 ||
            c.y > volpkg.getSliceHeight()) {
            continue;
        } else {
            break;
        }
    }

    // Get volume, write to hdf5 file
    int32_t radius = 3;
    auto volume = vol.getVoxelNeighborsCubic<uint16_t>({c.x, c.y, c.z}, radius);
    std::stringstream ss;
    ss << c.x << "_" << c.y << "_" << c.z << "_size" << 2 * radius + 1;
    const auto volumeFileName = ss.str() + "_volume.h5";
    const auto stFileName = ss.str() + "_st_eig.h5";
    writeVolumeToH5File(volumeFileName, volume);

    // Get ST & eigenvalues/vectors
    auto st = vol.structureTensorAtIndex(c.x, c.y, c.z, radius);
    auto pairs = vol.eigenPairsAtIndex(c.x, c.y, c.z, radius);
    writeStructureTensorEigToH5File(stFileName, st, pairs);
}

// Change polar coordinate to cartesian, taking into account offset center value
CartesianCoord polarToCartesian(const PolarCoord p, const int32_t z)
{
    return CartesianCoord(g_centerX + std::round(p.r * std::cos(p.theta)),
                          g_centerY + std::round(p.r * std::sin(p.theta)), z);
}

template <typename DType>
void writeVolumeToH5File(const std::string& filename,
                         const volcart::Tensor3D<DType>& volume)
{
    constexpr int32_t rank = 4;
    hsize_t dimsf[] = {1, static_cast<hsize_t>(volume.dz()),
                       static_cast<hsize_t>(volume.dy()),
                       static_cast<hsize_t>(volume.dx())};
    constexpr int32_t radius = 3;
    constexpr int32_t side = radius * 2 + 1;
    DType v[1][side][side][side];
    std::cout << "dx: " << volume.dx() << "\ndy: " << volume.dy()
              << "\ndz: " << volume.dz() << "\n";
    for (int32_t z = 0; z < volume.dz(); ++z) {
        for (int32_t y = 0; y < volume.dy(); ++y) {
            for (int32_t x = 0; x < volume.dx(); ++x) {
                v[0][z][y][x] = volume(x, y, z);
            }
        }
    }
    /*
    DType v[1][7][6][5];
    std::cout << "dx: " << volume.dx() << "\ndy: " << volume.dy()
              << "\ndz: " << volume.dz() << "\n";
    for (int32_t z = 0; z < 7; ++z) {
        for (int32_t y = 0; y < 6; ++y) {
            for (int32_t x = 0; x < 5; ++x) {
                v[0][z][y][x] = 1;
            }
        }
    }
    */

    try {
        H5::H5File f(filename, H5F_ACC_TRUNC);
        H5::DataSpace dspace(rank, dimsf);
        auto dset = f.createDataSet("data", H5::PredType::STD_U16LE, dspace);
        dset.write(v, H5::PredType::STD_U16LE);
    } catch (const H5::Exception& ex) {
        std::cerr << ex.getDetailMsg() << std::endl;
    }
}

void writeStructureTensorEigToH5File(const std::string& filename,
                                     const StructureTensor& st,
                                     const EigenPairs& pairs)
{
    // Dimensions for H5 data
    hsize_t stDims[] = {3, 3};
    int32_t stRank = 2;
    hsize_t eigvecDims[] = {3, 3};
    int32_t eigvecRank = 2;
    hsize_t eigvalDims[] = {3};
    int32_t eigvalRank = 1;

    // Raw buffers for data
    double stBuf[3][3] = {{st(0, 0), st(0, 1), st(0, 2)},
                          {st(1, 0), st(1, 1), st(1, 2)},
                          {st(2, 0), st(2, 1), st(2, 2)}};
    cv::Vec3d ev0 = pairs[0].second;
    cv::Vec3d ev1 = pairs[1].second;
    cv::Vec3d ev2 = pairs[2].second;
    double eigvecBuf[3][3] = {{ev0(0), ev0(1), ev0(2)},
                              {ev1(0), ev1(1), ev1(2)},
                              {ev2(0), ev2(1), ev2(2)}};
    double eigvalBuf[3] = {pairs[0].first, pairs[1].first, pairs[2].first};

    try {
        H5::H5File f(filename, H5F_ACC_TRUNC);

        // Write structure tensor
        H5::DataSpace stSpace(stRank, stDims);
        auto stDSet = f.createDataSet("StructureTensor",
                                      H5::PredType::NATIVE_FLOAT, stSpace);
        stDSet.write(stBuf, H5::PredType::NATIVE_FLOAT);

        // Write eigenvectors
        H5::DataSpace eigvecSpace(eigvecRank, eigvecDims);
        auto eigvecDSet = f.createDataSet(
            "Eigenvectors", H5::PredType::NATIVE_FLOAT, eigvecSpace);
        eigvecDSet.write(eigvecBuf, H5::PredType::NATIVE_FLOAT);

        // Write eigenvalues
        H5::DataSpace eigvalSpace(eigvalRank, eigvalDims);
        auto eigvalDSet = f.createDataSet(
            "Eigenvalues", H5::PredType::NATIVE_FLOAT, eigvalSpace);
        eigvalDSet.write(eigvalBuf, H5::PredType::NATIVE_FLOAT);
    } catch (const H5::Exception& ex) {
        std::cerr << ex.getDetailMsg() << std::endl;
    }
}

// Draw point on slice
void draw(const volcart::Volume& vol, const CartesianCoord c)
{
    auto slice = vol.getSliceData(c.z).clone();
    slice /= 255.0;
    slice.convertTo(slice, CV_8UC3);
    cv::cvtColor(slice, slice, CV_GRAY2BGR);
    cv::Point p{c.x, c.y};

    // Make filled in red circle (thickness == -1)
    cv::circle(slice, p, 10, cv::Scalar(0, 0, 0xFF), -1);
    cv::namedWindow("Slice", cv::WINDOW_NORMAL);
    cv::imshow("Slice", slice);
    cv::waitKey(0);
}
