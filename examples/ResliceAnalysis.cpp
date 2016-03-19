#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "VolumePkg.h"

const cv::Vec3d KVector(0, 0, 1);
const int32_t WIDTH = 32;
const int32_t HEIGHT = 32;
const cv::Scalar BGR_RED(0, 0, 0xFF);

inline double rad2deg(double rad) { return rad * 180.0 / M_PI; }

inline double deg2rad(double deg) { return deg * M_PI / 180.0; }

inline double vectorAngle(const cv::Vec3d v)
{
    return rad2deg(std::atan(v(1) / v(0)));
}

void drawReslice(cv::Mat reslice);

void drawSliceWithResliceVector(volcart::Volume& vpkg,
                                int32_t zSlice,
                                const cv::Vec3d resliceVector,
                                const cv::Point2i center);

int main(int argc, char** argv)
{
    if (argc < 5) {
        std::cerr << "Usage:" << std::endl;
        std::cerr << "    " << argv[0] << " volpkg cx cy cz" << std::endl;
        std::exit(1);
    }

    const std::string vpkgPath(argv[1]);
    const int32_t cx(std::stod(argv[2]));
    const int32_t cy(std::stod(argv[3]));
    const int32_t cz(std::stod(argv[4]));
    const cv::Vec3d center(cx, cy, cz);

    VolumePkg volpkg(vpkgPath);
    auto vol = volpkg.volume();

    // Get our normal estimate from the structure tensor
    auto pairs = vol.eigenPairsAt(cx, cy, cz, 5);
    auto normalVector = pairs[0].second;
    auto degrees = vectorAngle(normalVector);

    // Loop until key other than left/right arrow is pressed
    while (true) {

        // Perform reslice
        auto reslice =
            vol.reslice(center, normalVector, KVector, HEIGHT, WIDTH);

        // Draw
        drawReslice(reslice.draw());
        drawSliceWithResliceVector(vol, cz, normalVector, {cx, cy});

        // Adjust angle accordingly
        auto c = cv::waitKey();
        switch (c) {
        case 63235: {
            degrees += 1.0;
            auto rads = deg2rad(degrees);
            normalVector = {std::cos(rads), std::sin(rads), normalVector(2)};
            break;
        }
        case 63234: {
            degrees -= 1.0;
            auto rads = deg2rad(degrees);
            normalVector = {std::cos(rads), std::sin(rads), normalVector(2)};
            break;
        }
        default:
            return 0;
        }
    }
}

void drawReslice(cv::Mat reslice)
{
    const auto windowName = "Reslice view";
    cv::resize(reslice, reslice, {200, 200});
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::imshow(windowName, reslice);
}

void drawSliceWithResliceVector(volcart::Volume& v,
                                int32_t zSlice,
                                const cv::Vec3d resliceVector,
                                const cv::Point2i center)
{
    const auto windowName = "Slice with reslice vector";

    // Convert the CT slice to 8-bit RGB
    auto slice = v.getSliceDataCopy(zSlice);
    slice.convertTo(slice, CV_8UC3, 1.0 / 255.0);
    cv::cvtColor(slice, slice, CV_GRAY2BGR);

    // Draw scaled vector on it
    constexpr int32_t scale = 20;
    const cv::Point p2{cvRound(center.x + resliceVector(0) * scale),
                       cvRound(center.y + resliceVector(1) * scale)};
    cv::line(slice, center, p2, cv::Scalar(0, 0, 0xFF), 2);
    cv::resize(slice, slice, {slice.cols * 2, slice.rows * 2});

    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::imshow(windowName, slice);
}
