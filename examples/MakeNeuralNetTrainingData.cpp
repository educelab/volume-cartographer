#include <iostream>
#include <random>
#include <sstream>
#include <tuple>
#include <cmath>
#include <unordered_set>
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

// Hash specialization for CartesianCoord so unordered_set works
namespace std
{
template <>
struct hash<CartesianCoord> {
    size_t operator()(const CartesianCoord& c) const noexcept
    {
        return std::hash<int32_t>()(c.x) ^ std::hash<int32_t>()(c.y) ^
               std::hash<int32_t>()(c.z);
    }
};

// Equality specialization for CartesianCoord
template <>
struct equal_to<CartesianCoord> {
    bool operator()(const CartesianCoord& a, const CartesianCoord& b) const
        noexcept
    {
        return a.x == b.x && a.y == b.y && a.z == b.z;
    }
};
}

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
template <int32_t Radius>
void writeDataToH5File(const fs::path& filename,
                       const volcart::Tensor3D<uint16_t>& volume,
                       const StructureTensor st, const EigenPairs& pairs);

const StructureTensor zero(0, 0, 0, 0, 0, 0, 0, 0, 0);

// Center of the dataset on XY-plane
int32_t g_centerX;
int32_t g_centerY;

int main(int argc, char** argv)
{
    if (argc < 11) {
        std::cerr
            << "Usage:\n"
            << "    " << argv[0]
            << " volumepkg N outer-r inner-r centerX centerY zmin zmax st-r"
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
    const int32_t stRadius = std::stoi(std::string(argv[9]));
    const auto outputDir = fs::path(argv[10]);

    // DEBUG
    std::cout << "nsamples:  " << nsamples << "\n";
    std::cout << "outerRad:  " << outerRadius << "\n";
    std::cout << "innerRad:  " << innerRadius << "\n";
    std::cout << "centerX:   " << g_centerX << "\n";
    std::cout << "centerY:   " << g_centerY << "\n";
    std::cout << "zMin:      " << zmin << "\n";
    std::cout << "zMax:      " << zmax << "\n";
    std::cout << "stRadius:  " << stRadius << "\n";
    std::cout << "outputDir: " << outputDir << "\n";

    // Make output directories
    fs::create_directory(outputDir);
    fs::create_directory(outputDir / "3");
    fs::create_directory(outputDir / "7");
    fs::create_directory(outputDir / "15");

    const volcart::Volume vol = volpkg.volume();

    // Make generator and distributions
    std::random_device rd;
    std::default_random_engine e(rd());
    std::uniform_real_distribution<double> rDist(innerRadius, outerRadius);
    std::uniform_real_distribution<double> thetaDist(0, 2 * M_PI);
    std::uniform_int_distribution<int32_t> zDist(zmin, zmax + 1);

    // Start generating
    CartesianCoord c;
    int64_t i = 2000;
    while (i < nsamples) {
        double theta = thetaDist(e);
        double r = rDist(e);
        int32_t z = zDist(e);
        c = polarToCartesian({r, theta}, z);

        // Volume bounds checking
        if (c.x < 0 || c.x >= volpkg.getSliceWidth() || c.y < 0 ||
            c.y >= volpkg.getSliceHeight()) {
            continue;
        }

        // Generate structure tensor and eigenpairs once per point
        auto st = vol.structureTensorAtIndex(c.x, c.y, c.z, stRadius);
        if (st == zero) {
            continue;
        }
        auto pairs = vol.eigenPairsAtIndex(c.x, c.y, c.z, stRadius);

        // Start constructing filename
        std::stringstream ss;
        ss << c.x << "_" << c.y << "_" << c.z << "_size";
        auto baseFilename = ss.str();

        // Generate volumes of radius 3, 7, and 15
        cv::Point3i p = {c.x, c.y, c.z};
        auto volume3 = vol.getVoxelNeighborsCubic<uint16_t>(p, 3);
        auto outfile3 = outputDir / std::to_string(3) /
                        (baseFilename + std::to_string(3) + ".h5");
        auto volume7 = vol.getVoxelNeighborsCubic<uint16_t>(p, 7);
        auto outfile7 = outputDir / std::to_string(7) /
                        (baseFilename + std::to_string(7) + ".h5");
        auto volume15 = vol.getVoxelNeighborsCubic<uint16_t>(p, 15);
        auto outfile15 = outputDir / std::to_string(15) /
                         (baseFilename + std::to_string(15) + ".h5");

        ++i;
        if (i % 1000 == 0) {
            std::cout << "npoints: " << i << "\n";
        }

        // Do writes
        writeDataToH5File<3>(outfile3, volume3, st, pairs);
        writeDataToH5File<7>(outfile7, volume7, st, pairs);
        writeDataToH5File<15>(outfile15, volume15, st, pairs);
    }
}

// Change polar coordinate to cartesian, taking into account offset center value
CartesianCoord polarToCartesian(const PolarCoord p, const int32_t z)
{
    return CartesianCoord(g_centerX + std::round(p.r * std::cos(p.theta)),
                          g_centerY + std::round(p.r * std::sin(p.theta)), z);
}

template <int32_t Radius>
void writeDataToH5File(const fs::path& filename,
                       const volcart::Tensor3D<uint16_t>& volume,
                       const StructureTensor st, const EigenPairs& pairs)
{
    constexpr int32_t volRank = 4;
    hsize_t volDims[] = {1, static_cast<hsize_t>(volume.dz()),
                         static_cast<hsize_t>(volume.dy()),
                         static_cast<hsize_t>(volume.dx())};
    int32_t side = 2 * Radius + 1;
    uint16_t v[1][side][side][side];
    for (int32_t z = 0; z < volume.dz(); ++z) {
        for (int32_t y = 0; y < volume.dy(); ++y) {
            for (int32_t x = 0; x < volume.dx(); ++x) {
                v[0][z][y][x] = volume(x, y, z);
            }
        }
    }

    // Dimensions for structure tensor & eigenpairs data
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
        // Write volume
        H5::H5File f(filename.string(), H5F_ACC_TRUNC);
        H5::DataSpace volumeSpace(volRank, volDims);
        auto volDSet =
            f.createDataSet("volume", H5::PredType::STD_U16LE, volumeSpace);
        volDSet.write(v, H5::PredType::STD_U16LE);

        // Write structure tensor
        H5::DataSpace stSpace(stRank, stDims);
        auto stDSet = f.createDataSet("structure_tensor",
                                      H5::PredType::NATIVE_FLOAT, stSpace);
        stDSet.write(stBuf, H5::PredType::NATIVE_FLOAT);

        // Write eigenvectors
        H5::DataSpace eigvecSpace(eigvecRank, eigvecDims);
        auto eigvecDSet = f.createDataSet(
            "eigenvectors", H5::PredType::NATIVE_FLOAT, eigvecSpace);
        eigvecDSet.write(eigvecBuf, H5::PredType::NATIVE_FLOAT);

        // Write eigenvalues
        H5::DataSpace eigvalSpace(eigvalRank, eigvalDims);
        auto eigvalDSet = f.createDataSet(
            "eigenvalues", H5::PredType::NATIVE_FLOAT, eigvalSpace);
        eigvalDSet.write(eigvalBuf, H5::PredType::NATIVE_FLOAT);

        f.close();

    } catch (const H5::Exception& ex) {
        std::cerr << ex.getDetailMsg() << std::endl;
    }
}
