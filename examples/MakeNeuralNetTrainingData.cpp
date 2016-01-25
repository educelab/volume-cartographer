#include <iostream>
#include <random>
#include <tuple>
#include <boost/filesystem.hpp>
#include <itkImage.h>
#include "volumepkg.h"

namespace fs = boost::filesystem;

using CartesianCoord = std::tuple<int32_t, int32_t, int32_t>;
using PolarCoord = std::tuple<double, double>;

int32_t g_centerX;  //
int32_t g_centerY;  //

int main(int argc, char** argv)
{
    if (argc < 8) {
        std::cerr << "Usage:\n"
                  << "    " << argv[0]
                  << " volumepkg N outer-r inner-r centerX centerY zmin zmax "
                     "output-dir\n";
        std::exit(1);
    }
    auto volpkgPath = fs::path(argv[1]);
    int32_t nsamples = std::stoi(std::string(argv[2]));
    int32_t outerRadius = std::stoi(std::string(argv[3]));
    int32_t innerRadius = std::stoi(std::string(argv[4]));
    g_centerX = std::stoi(std::string(argv[5]));
    g_centerY = std::stoi(std::string(argv[6]));
    int32_t zmin = std::stoi(std::string(argv[7]));
    int32_t zmax = std::stoi(std::string(argv[8]));
    auto outputDir = fs::path(argv[9]);

    // Make generator and distributions
    std::random_device r;
    std::default_random_engine e1(r());
    std::uniform_real_distribution<> rDist(innerRadius, outerRadius);
    std::uniform_real_distribution<> thetaDist(0, 1);
}

// Change polar coordinate to cartesian, taking into account offset center value
CartesianCoord polarToCartesian(const PolarCoord coord, const int32_t z)
{
    double r, theta;
    std::tie(r, theta) = coord;
    return std::make_tuple(g_centerX + std::round(r * std::cos(theta)),
                           g_centerY + std::round(r * std::sin(theta)), z);
}
