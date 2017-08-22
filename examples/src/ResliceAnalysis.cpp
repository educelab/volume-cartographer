#include <cmath>
#include <iostream>
#include <random>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "vc/core/types/VolumePkg.hpp"

const cv::Vec3d KVector(0, 0, 1);
const int32_t WIDTH = 64;
const int32_t HEIGHT = 64;
const cv::Scalar BGR_RED(0, 0, 0xFF);
const double RadianStep = 3 * M_PI / 180.0;

inline double rad2deg(double rad) { return rad * 180.0 / M_PI; }

inline double deg2rad(double deg) { return deg * M_PI / 180.0; }

inline std::pair<double, double> CartesianToSpherical(cv::Vec3d v)
{
    return {std::atan(v(1) / v(0)), std::acos(v(2))};
}

inline cv::Vec3d SphericalToCartesian(double theta, double phi)
{
    return {std::sin(phi) * std::cos(theta), std::sin(phi) * std::sin(theta),
            std::cos(phi)};
}

void drawReslice(cv::Mat reslice);

void drawSliceWithResliceVector(
    volcart::Volume::Pointer volume,
    const cv::Vec3d resliceVector,
    const cv::Vec3d center);

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "Usage:" << std::endl;
        std::cerr << "    " << argv[0] << " volpkg [cx cy cz]" << std::endl;
        std::cerr << std::endl;
        std::cerr << "Controls:" << std::endl;
        std::cerr << "    Left arrow:  decrease theta" << std::endl;
        std::cerr << "    Right arrow: increase theta" << std::endl;
        std::cerr << "    Up arrow:    increase phi" << std::endl;
        std::cerr << "    Down arrow:  decrease phi" << std::endl;
        std::cerr << std::endl;
        std::cerr << "With no [cx, cy, cz], defaults to generating a random "
                     "voxel coordinate in the volume"
                  << std::endl;
        std::exit(1);
    }

    const std::string vpkgPath(argv[1]);
    volcart::VolumePkg volpkg(vpkgPath);
    auto vol = volpkg.volume();

    // Set locations to do arbitrary reslicing
    int cx, cy, cz;
    if (argc > 2) {
        cx = std::stoi(argv[2]);
        cy = std::stoi(argv[3]);
        cz = std::stoi(argv[4]);
    } else {
        // Generate random starting point
        std::random_device rd;
        std::default_random_engine eng(rd());
        std::uniform_int_distribution<> xdist(0, vol->sliceWidth());
        std::uniform_int_distribution<> ydist(0, vol->sliceHeight());
        std::uniform_int_distribution<> zdist(0, vol->numSlices());
        cx = xdist(eng);
        cy = ydist(eng);
        cz = zdist(eng);
    }

    const cv::Vec3d center(cx, cy, cz);

    // Get our normal estimate from the structure tensor
    auto pairs = vol->eigenPairsAt(cx, cy, cz, 5);
    auto normalVector = pairs[0].second;
    double theta, phi;
    std::tie(theta, phi) = CartesianToSpherical(normalVector);

    // Loop until key other than left/right arrow is pressed
    while (true) {

        // Figure out what the orthogonal vector should be given the reslice
        // vector. The vectors should be orthogonal (i.e. dot product should be
        // 0). Depending on which quadrant the reslice vector is in, orthogonal
        // vector's theta should be the same as reslice vector's or on the
        // opposite side of the circle in the XY plane.
        const double orthoTheta = (phi <= M_PI / 2 ? theta + M_PI : theta);
        auto orthogonalVector =
            SphericalToCartesian(orthoTheta, std::abs(phi - M_PI / 2.0));

        // Do reslice
        auto reslice =
            vol->reslice(center, normalVector, orthogonalVector, HEIGHT, WIDTH);

        // Draw
        drawReslice(reslice.draw());
        drawSliceWithResliceVector(vol, normalVector, center);

        // Adjust angle accordingly
        // Use polar coordinates:
        //     theta: azimuth (rotation around XY plane)
        //     phi: inclination (angle from Z axis)
        switch (cv::waitKey()) {

            // Right arrow - increase phi
            case 63235:
                theta += RadianStep;
                break;

            // Left arrow - decrease phi
            case 63234:
                theta -= RadianStep;
                break;

            // Up arrow - increase theta
            case 63232:
                phi = (phi >= M_PI ? M_PI : phi + RadianStep);
                break;

            // Down arrow - decrease theta
            case 63233:
                phi = (phi <= 0 ? 0 : phi - RadianStep);
                break;

            default:
                return 0;
        }

        // Calculate new vector
        normalVector = SphericalToCartesian(theta, phi);
    }
}

void drawReslice(cv::Mat reslice)
{
    const auto windowName = "Reslice view";
    cv::resize(reslice, reslice, {200, 200});
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::imshow(windowName, reslice);
}

void drawSliceWithResliceVector(
    volcart::Volume::Pointer v,
    const cv::Vec3d resliceVector,
    const cv::Vec3d center)
{
    const auto windowName = "Slice with reslice vector";

    // Convert the CT slice to 8-bit RGB
    auto slice = v->getSliceDataCopy(int32_t(center(2)));
    slice.convertTo(slice, CV_8UC3, 1.0 / 255.0);
    cv::cvtColor(slice, slice, cv::COLOR_GRAY2BGR);

    // Draw scaled vector on it
    constexpr int32_t scale = 20;
    const double zProjection = (1 - std::abs(resliceVector(2)));
    const auto p2 = center + resliceVector * zProjection * scale;
    cv::line(
        slice, {int32_t(center(0)), int32_t(center(1))},
        {int32_t(p2(0)), int32_t(p2(1))}, cv::Scalar(0, 0, 0xFF), 2);
    // cv::resize(slice, slice, {slice.cols / 2, slice.rows / 2});

    cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
    cv::imshow(windowName, slice);
}
