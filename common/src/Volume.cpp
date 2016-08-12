#include "common/types/Volume.h"
#include <iomanip>
#include <iostream>
#include <memory>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>

using namespace volcart;
namespace fs = boost::filesystem;

StructureTensor tensorize(const cv::Vec3d vec);

std::unique_ptr<double[]> makeUniformGaussianField(const int32_t radius);

// Trilinear Interpolation: Particles are not required
// to be at integer positions so we estimate their
// normals with their neighbors's known normals.
//
// formula from http://paulbourke.net/miscellaneous/interpolation/
uint16_t Volume::interpolateAt(const Voxel point) const
{
    double int_part;
    const double dx = std::modf(point(0), &int_part);
    const int32_t x0 = int32_t(int_part);
    const int32_t x1 = x0 + 1;
    const double dy = std::modf(point(1), &int_part);
    const int32_t y0 = int32_t(int_part);
    const int32_t y1 = y0 + 1;
    const double dz = std::modf(point(2), &int_part);
    const int32_t z0 = int32_t(int_part);
    const int32_t z1 = z0 + 1;

    // insert safety net
    if (x0 < 0 || y0 < 0 || z0 < 0 || x1 >= sliceWidth_ || y1 >= sliceHeight_ ||
        z1 >= numSlices_) {
        return 0;
    }

    // from: https://en.wikipedia.org/wiki/Trilinear_interpolation
    const auto c00 =
        intensityAt(x0, y0, z0) * (1 - dx) + intensityAt(x1, y0, z0) * dx;
    const auto c10 =
        intensityAt(x0, y1, z0) * (1 - dx) + intensityAt(x1, y0, z0) * dx;
    const auto c01 =
        intensityAt(x0, y0, z1) * (1 - dx) + intensityAt(x1, y0, z1) * dx;
    const auto c11 =
        intensityAt(x0, y1, z1) * (1 - dx) + intensityAt(x1, y1, z1) * dx;

    const auto c0 = c00 * (1 - dy) + c10 * dy;
    const auto c1 = c01 * (1 - dy) + c11 * dy;

    const auto c = c0 * (1 - dz) + c1 * dz;
    return uint16_t(cvRound(c));
}

const cv::Mat& Volume::getSliceData(int32_t index) const
{
    // Take advantage of caching layer
    try {
        return cache_.get(index);
    } catch (const std::exception& ex) {
        const auto slicePath = getSlicePath(index);
        const cv::Mat sliceImg = cv::imread(slicePath.string(), -1);

        // Put into cache so we can use it later
        cache_.put(index, sliceImg);
        return cache_.get(index);
    }
}

cv::Mat Volume::getSliceDataCopy(int32_t index) const
{
    return getSliceData(index).clone();
}

// Data Assignment
bool Volume::setSliceData(int32_t index, const cv::Mat& slice)
{
    if (index >= numSlices_) {
        std::cerr << "ERROR: Atttempted to save a slice image to an out of "
                     "bounds index."
                  << std::endl;
        return false;
    }

    const auto filepath = getSlicePath(index);
    cv::imwrite(filepath.string(), slice);
    return true;
}

fs::path Volume::getSlicePath(int32_t index) const
{
    std::stringstream ss;
    ss << std::setw(numSliceCharacters_) << std::setfill('0') << index
       << ".tif";
    return slicePath_ / ss.str();
}

Slice Volume::reslice(const Voxel center,
                      const cv::Vec3d xvec,
                      const cv::Vec3d yvec,
                      int32_t width,
                      int32_t height) const
{
    const auto xnorm = cv::normalize(xvec);
    const auto ynorm = cv::normalize(yvec);
    const auto origin = center - ((width / 2) * xnorm + (height / 2) * ynorm);

    cv::Mat m(height, width, CV_16UC1);
    for (int32_t h = 0; h < height; ++h) {
        for (int32_t w = 0; w < width; ++w) {
            m.at<uint16_t>(h, w) =
                interpolateAt(origin + (h * ynorm) + (w * xnorm));
        }
    }

    return Slice(m, origin, xnorm, ynorm);
}

StructureTensor Volume::structureTensorAt(int32_t vx,
                                          int32_t vy,
                                          int32_t vz,
                                          int32_t voxelRadius,
                                          int32_t gradientKernelSize) const
{
    // Safety checks
    assert(vx >= 0 && vx < sliceWidth_ &&
           "x must be in range [0, slice_width]");
    assert(vy >= 0 && vy < sliceHeight_ &&
           "y must be in range [0, slice_height]");
    assert(vz >= 0 && vz < numSlices_ && "z must be in range [0, #slices]");
    assert(gradientKernelSize >= 3 &&
           "gradient kernel size must be at least 3");

    // Get voxels in radius voxelRadius around voxel (x, y, z)
    auto v = getVoxelNeighborsCubic<double>({vx, vy, vz}, voxelRadius);

    // Normalize voxel neighbors to [0, 1]
    for (int32_t z = 0; z < v.dz; ++z) {
        v.xySlice(z) /= std::numeric_limits<uint16_t>::max();
    }

    // Get gradient of volume
    auto gradientField = volumeGradient(v, gradientKernelSize);

    // Modulate by gaussian distribution (element-wise) and sum
    auto gaussianField = makeUniformGaussianField(voxelRadius);
    StructureTensor sum(0, 0, 0, 0, 0, 0, 0, 0, 0);
    for (int32_t z = 0; z < v.dz; ++z) {
        for (int32_t y = 0; y < v.dy; ++y) {
            for (int32_t x = 0; x < v.dx; ++x) {
                sum += gaussianField[z * v.dy * v.dx + y * v.dx + x] *
                       tensorize(gradientField(x, y, z));
            }
        }
    }

    cv::Mat matSum(sum);
    matSum /= v.dx * v.dy * v.dz;
    return StructureTensor(matSum);
}

StructureTensor Volume::interpolatedStructureTensorAt(
    double vx,
    double vy,
    double vz,
    int32_t voxelRadius,
    int32_t gradientKernelSize) const
{
    // Safety checks
    assert(vx >= 0 && vx < sliceWidth_ &&
           "x must be in range [0, slice_width]");
    assert(vy >= 0 && vy < sliceHeight_ &&
           "y must be in range [0, slice_height]");
    assert(vz >= 0 && vz < numSlices_ && "z must be in range [0, #slices]");
    assert(gradientKernelSize >= 3 &&
           "gradient kernel size must be at least 3");

    // Get voxels in radius voxelRadius around voxel (x, y, z)
    auto v =
        getVoxelNeighborsCubicInterpolated<double>({vx, vy, vz}, voxelRadius);

    // Get gradient of volume
    auto gradientField = volumeGradient(v, gradientKernelSize);

    // Modulate by gaussian distribution (element-wise) and sum
    auto gaussianField = makeUniformGaussianField(voxelRadius);
    StructureTensor sum(0, 0, 0, 0, 0, 0, 0, 0, 0);
    for (int32_t z = 0; z < v.dz; ++z) {
        for (int32_t y = 0; y < v.dy; ++y) {
            for (int32_t x = 0; x < v.dx; ++x) {
                sum += gaussianField[z * v.dy * v.dx + y * v.dx + x] *
                       tensorize(gradientField(x, y, z));
            }
        }
    }

    cv::Mat matSum(sum);
    matSum /= v.dx * v.dy * v.dz;
    return StructureTensor(matSum);
}

EigenPairs Volume::eigenPairsAt(int32_t x,
                                int32_t y,
                                int32_t z,
                                int32_t voxelRadius,
                                int32_t gradientKernelSize) const
{
    auto st = structureTensorAt(x, y, z, voxelRadius, gradientKernelSize);
    cv::Vec3d eigenValues;
    cv::Matx33d eigenVectors;
    cv::eigen(st, eigenValues, eigenVectors);
    auto row0 = eigenVectors.row(0);
    auto row1 = eigenVectors.row(1);
    auto row2 = eigenVectors.row(2);
    EigenVector e0{row0(0), row0(1), row0(2)};
    EigenVector e1{row1(0), row1(1), row1(2)};
    EigenVector e2{row2(0), row2(1), row2(2)};
    return {
        std::make_pair(eigenValues(0), e0), std::make_pair(eigenValues(1), e1),
        std::make_pair(eigenValues(2), e2),
    };
}

EigenPairs Volume::interpolatedEigenPairsAt(double x,
                                            double y,
                                            double z,
                                            int32_t voxelRadius,
                                            int32_t gradientKernelSize) const
{
    auto st =
        interpolatedStructureTensorAt(x, y, z, voxelRadius, gradientKernelSize);
    cv::Vec3d eigenValues;
    cv::Matx33d eigenVectors;
    cv::eigen(st, eigenValues, eigenVectors);
    auto row0 = eigenVectors.row(0);
    auto row1 = eigenVectors.row(1);
    auto row2 = eigenVectors.row(2);
    EigenVector e0{row0(0), row0(1), row0(2)};
    EigenVector e1{row1(0), row1(1), row1(2)};
    EigenVector e2{row2(0), row2(1), row2(2)};
    return {
        std::make_pair(eigenValues(0), e0), std::make_pair(eigenValues(1), e1),
        std::make_pair(eigenValues(2), e2),
    };
}

StructureTensor tensorize(const cv::Vec3d gradient)
{
    double ix = gradient(0);
    double iy = gradient(1);
    double iz = gradient(2);
    // clang-format off
    return StructureTensor(ix * ix, ix * iy, ix * iz,
                           ix * iy, iy * iy, iy * iz,
                           ix * iz, iy * iz, iz * iz);
    // clang-format on
}

Tensor3D<cv::Vec3d> Volume::volumeGradient(const Tensor3D<double>& v,
                                           int32_t gradientKernelSize) const
{
    // Limitation of OpenCV: Kernel size must be 1, 3, 5, or 7
    assert((gradientKernelSize == 1 || gradientKernelSize == 3 ||
            gradientKernelSize == 5 || gradientKernelSize == 7) &&
           "gradientKernelSize must be one of [1, 3, 5, 7]");

    // Calculate gradient field around specified voxel
    Tensor3D<cv::Vec3d> gradientField{v.dx, v.dy, v.dz};

    // First do XY gradients
    for (int32_t z = 0; z < v.dz; ++z) {
        const cv::Mat_<double> xGradient =
            gradient(v.xySlice(z), GradientAxis::X, gradientKernelSize);
        const cv::Mat_<double> yGradient =
            gradient(v.xySlice(z), GradientAxis::Y, gradientKernelSize);
        for (int32_t y = 0; y < v.dy; ++y) {
            for (int32_t x = 0; x < v.dx; ++x) {
                gradientField(x, y, z) = {xGradient(y, x), yGradient(y, x), 0};
            }
        }
    }

    // Then Z gradients
    for (int32_t layer = 0; layer < v.dy; ++layer) {
        const cv::Mat_<double> zGradient =
            gradient(v.xzSlice(layer), GradientAxis::Y, gradientKernelSize);
        for (int32_t z = 0; z < v.dz; ++z) {
            for (int32_t x = 0; x < v.dx; ++x) {
                gradientField(x, layer, z)(2) = zGradient(z, x);
            }
        }
    }

    return gradientField;
}

// Helper function to calculate gradient using best choice for given kernel
// size. If the kernel size is 3, uses the Scharr() operator to calculate the
// gradient which is more accurate than 3x3 Sobel operator
cv::Mat_<double> Volume::gradient(const cv::Mat_<double>& input,
                                  GradientAxis axis,
                                  int32_t ksize) const
{
    // OpenCV params for gradients
    // XXX Revisit this and see if changing these makes a big difference
    constexpr double SCALE = 1;
    constexpr double DELTA = 0;

    cv::Mat_<double> grad(input.rows, input.cols);

    switch (axis) {
    case GradientAxis::X:
        if (ksize == 3) {
            cv::Scharr(input, grad, CV_64F, 1, 0, SCALE, DELTA,
                       cv::BORDER_REPLICATE);
        } else {
            cv::Sobel(input, grad, CV_64F, 1, 0, ksize, SCALE, DELTA,
                      cv::BORDER_REPLICATE);
        }
        break;
    case GradientAxis::Y:
        if (ksize == 3) {
            cv::Scharr(input, grad, CV_64F, 0, 1, SCALE, DELTA,
                       cv::BORDER_REPLICATE);
        } else {
            cv::Sobel(input, grad, CV_64F, 0, 1, ksize, SCALE, DELTA,
                      cv::BORDER_REPLICATE);
        }
        break;
    }
    return grad;
}

std::unique_ptr<double[]> makeUniformGaussianField(int32_t radius)
{
    const int32_t sideLength = 2 * radius + 1;
    const int32_t fieldSize = sideLength * sideLength * sideLength;
    auto field = std::unique_ptr<double[]>(new double[fieldSize]);
    double sum = 0;
    const double sigma = 1.0;
    const double sigma3 = sigma * sigma * sigma;
    const double N = 1.0 / (sigma3 * std::pow(2 * M_PI, 3.0 / 2.0));

    // Fill field
    for (int32_t z = -radius; z <= radius; ++z) {
        for (int32_t y = -radius; y <= radius; ++y) {
            for (int32_t x = -radius; x <= radius; ++x) {
                double val = std::exp(-(x * x + y * y + z * z));
                // clang-format off
                field[(z + radius) * sideLength * sideLength +
                      (y + radius) * sideLength +
                      (x + radius)] = N * val;
                // clang-format on
                sum += val;
            }
        }
    }

    // Normalize
    for (int32_t i = 0; i < sideLength * sideLength * sideLength; ++i) {
        field[i] /= sum;
    }

    return field;
}
