#include <iostream>
#include <random>
#include <tuple>
#include <cmath>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>
#include "volumepkg.h"

namespace fs = boost::filesystem;

struct CartesianCoord {
    int32_t x, y, z;
    CartesianCoord(int32_t x, int32_t y, int32_t z) : x(x), y(y), z(z) {}
};
std::ostream& operator<<(std::ostream& s, const CartesianCoord& c)
{
    return s << "[" << c.x << ", " << c.y << ", " << c.z << "]";
}

struct PolarCoord {
    double r, theta;
    PolarCoord(double r, double theta) : r(r), theta(theta) {}
};
std::ostream& operator<<(std::ostream& s, const PolarCoord& p)
{
    return s << "[" << p.r << ", " << p.theta << "]";
}

CartesianCoord polarToCartesian(const PolarCoord coord, const int32_t z);
void draw(const volcart::Volume& vol, const CartesianCoord c);

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
    while (true) {
        double theta = thetaDist(e);
        double r = rDist(e);
        int32_t z = zDist(e);
        std::cout << "r: " << r << ", theta: " << theta << ", z: " << z
                  << std::endl;
        CartesianCoord c = polarToCartesian({r, theta}, z);

        // Volume bounds checking
        if (c.x < 0 || c.x > volpkg.getSliceWidth() || c.y < 0 ||
            c.y > volpkg.getSliceHeight()) {
            continue;
        }

        std::cout << c << std::endl;

        draw(vol, c);
    }
}

// Change polar coordinate to cartesian, taking into account offset center value
CartesianCoord polarToCartesian(const PolarCoord p, const int32_t z)
{
    return CartesianCoord(g_centerX + std::round(p.r * std::cos(p.theta)),
                          g_centerY + std::round(p.r * std::sin(p.theta)), z);
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
