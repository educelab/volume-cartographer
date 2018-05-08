#include "vc/core/types/Volume.hpp"

#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "vc/core/util/FloatComparison.hpp"

using namespace volcart;
namespace fs = boost::filesystem;

static const fs::path METADATA_FILE = "meta.json";

// Load a Volume from disk
Volume::Volume(fs::path path) : path_(std::move(path))
{
    metadata_ = volcart::Metadata(path_ / METADATA_FILE);
    sliceWidth_ = metadata_.get<int>("width");
    sliceHeight_ = metadata_.get<int>("height");
    numSlices_ = metadata_.get<int>("slices");
    numSliceCharacters_ = std::to_string(numSlices_).size();
}

// Setup a Volume from a folder of slices
Volume::Volume(fs::path path, std::string uuid, std::string name)
    : path_(std::move(path))
    , numSlices_(0)
    , sliceWidth_(0)
    , sliceHeight_(0)
    , numSliceCharacters_(0)
{
    metadata_.setPath((path_ / METADATA_FILE));
    metadata_.set("uuid", std::move(uuid));
    metadata_.set("name", std::move(name));
    metadata_.set("type", "vol");
    metadata_.set("width", sliceWidth_);
    metadata_.set("height", sliceHeight_);
    metadata_.set("slices", numSlices_);
    metadata_.set("voxelsize", double{});
    metadata_.set("min", double{});
    metadata_.set("max", double{});
}

// Load a Volume from disk, return a pointer
Volume::Pointer Volume::New(fs::path path)
{
    return std::make_shared<Volume>(path);
}

// Set a Volume from a folder of slices, return a pointer
Volume::Pointer Volume::New(fs::path path, std::string uuid, std::string name)
{
    return std::make_shared<Volume>(path, uuid, name);
}

StructureTensor Tensorize(cv::Vec3d gradient);

std::unique_ptr<double[]> MakeUniformGaussianField(int radius);

// Trilinear Interpolation: Particles are not required
// to be at integer positions so we estimate their
// normals with their neighbors's known normals.
//
// formula from http://paulbourke.net/miscellaneous/interpolation/
uint16_t Volume::interpolateAt(const Voxel& point) const
{
    double intPart;
    double dx = std::modf(point(0), &intPart);
    auto x0 = static_cast<int>(intPart);
    int x1 = x0 + 1;
    double dy = std::modf(point(1), &intPart);
    auto y0 = static_cast<int>(intPart);
    int y1 = y0 + 1;
    double dz = std::modf(point(2), &intPart);
    auto z0 = static_cast<int>(intPart);
    int z1 = z0 + 1;

    // insert safety net
    if (!isInBounds(point)) {
        return 0;
    }

    // from: https://en.wikipedia.org/wiki/Trilinear_interpolation
    auto c00 =
        intensityAt(x0, y0, z0) * (1 - dx) + intensityAt(x1, y0, z0) * dx;
    auto c10 =
        intensityAt(x0, y1, z0) * (1 - dx) + intensityAt(x1, y0, z0) * dx;
    auto c01 =
        intensityAt(x0, y0, z1) * (1 - dx) + intensityAt(x1, y0, z1) * dx;
    auto c11 =
        intensityAt(x0, y1, z1) * (1 - dx) + intensityAt(x1, y1, z1) * dx;

    auto c0 = c00 * (1 - dy) + c10 * dy;
    auto c1 = c01 * (1 - dy) + c11 * dy;

    auto c = c0 * (1 - dz) + c1 * dz;
    return static_cast<uint16_t>(cvRound(c));
}

const cv::Mat& Volume::getSliceData(int index) const
{
    // Take advantage of caching layer
    try {
        return cache_.get(index);
    } catch (const std::exception& ex) {
        auto slicePath = getSlicePath(index);
        auto sliceImg = cv::imread(slicePath.string(), -1);

        // Put into cache so we can use it later
        cache_.put(index, sliceImg);
        return cache_.get(index);
    }
}

cv::Mat Volume::getSliceDataCopy(int index) const
{
    return getSliceData(index).clone();
}

// Data Assignment
void Volume::setSliceData(int index, const cv::Mat& slice)
{
    if (index >= numSlices_) {
        auto msg = "Attempted to save a slice to an out-of-bounds index.";
        throw std::range_error(msg);
    }

    auto filepath = getSlicePath(index);
    cv::imwrite(filepath.string(), slice);
}

fs::path Volume::getSlicePath(int index) const
{
    std::stringstream ss;
    ss << std::setw(numSliceCharacters_) << std::setfill('0') << index
       << ".tif";
    return path_ / ss.str();
}

Reslice Volume::reslice(
    const Voxel& center,
    const cv::Vec3d& xvec,
    const cv::Vec3d& yvec,
    int width,
    int height) const
{
    if (!isInBounds(center)) {
        throw std::range_error("center not in bounds");
    }

    auto xnorm = cv::normalize(xvec);
    auto ynorm = cv::normalize(yvec);
    auto origin = center - ((width / 2) * xnorm + (height / 2) * ynorm);

    cv::Mat m(height, width, CV_16UC1);
    for (int h = 0; h < height; ++h) {
        for (int w = 0; w < width; ++w) {
            m.at<uint16_t>(h, w) =
                interpolateAt(origin + (h * ynorm) + (w * xnorm));
        }
    }

    return Reslice(m, origin, xnorm, ynorm);
}

StructureTensor Volume::structureTensorAt(
    int vx, int vy, int vz, int voxelRadius, int gradientKernelSize) const
{
    // Safety checks
    assert(
        vx >= 0 && vx < sliceWidth_ && "x must be in range [0, slice_width]");
    assert(
        vy >= 0 && vy < sliceHeight_ && "y must be in range [0, slice_height]");
    assert(vz >= 0 && vz < numSlices_ && "z must be in range [0, #slices]");
    assert(
        gradientKernelSize >= 3 && "gradient kernel size must be at least 3");

    // Get voxels in radius voxelRadius around voxel (x, y, z)
    auto v = getVoxelNeighborsCubic<double>({vx, vy, vz}, voxelRadius);

    // Normalize voxel neighbors to [0, 1]
    for (size_t z = 0; z < v.dz(); ++z) {
        v.xySlice(z) /= std::numeric_limits<uint16_t>::max();
    }

    // Get gradient of volume
    auto gradientField = volume_gradient_(v, gradientKernelSize);

    // Modulate by gaussian distribution (element-wise) and sum
    auto gaussianField = MakeUniformGaussianField(voxelRadius);
    StructureTensor sum(0, 0, 0, 0, 0, 0, 0, 0, 0);
    for (size_t z = 0; z < v.dz(); ++z) {
        for (size_t y = 0; y < v.dy(); ++y) {
            for (size_t x = 0; x < v.dx(); ++x) {
                sum += gaussianField[z * v.dy() * v.dx() + y * v.dx() + x] *
                       Tensorize(gradientField(x, y, z));
            }
        }
    }

    cv::Mat matSum(sum);
    matSum /= v.dx() * v.dy() * v.dz();
    return StructureTensor{matSum};
}

StructureTensor Volume::interpolatedStructureTensorAt(
    double vx,
    double vy,
    double vz,
    int voxelRadius,
    int gradientKernelSize) const
{
    // Safety checks
    assert(
        vx >= 0 && vx < sliceWidth_ && "x must be in range [0, slice_width]");
    assert(
        vy >= 0 && vy < sliceHeight_ && "y must be in range [0, slice_height]");
    assert(vz >= 0 && vz < numSlices_ && "z must be in range [0, #slices]");
    assert(
        gradientKernelSize >= 3 && "gradient kernel size must be at least 3");

    // Get voxels in radius voxelRadius around voxel (x, y, z)
    auto v =
        getVoxelNeighborsCubicInterpolated<double>({vx, vy, vz}, voxelRadius);

    // Get gradient of volume
    auto gradientField = volume_gradient_(v, gradientKernelSize);

    // Modulate by gaussian distribution (element-wise) and sum
    auto gaussianField = MakeUniformGaussianField(voxelRadius);
    StructureTensor sum(0, 0, 0, 0, 0, 0, 0, 0, 0);
    for (size_t z = 0; z < v.dz(); ++z) {
        for (size_t y = 0; y < v.dy(); ++y) {
            for (size_t x = 0; x < v.dx(); ++x) {
                sum += gaussianField[z * v.dy() * v.dx() + y * v.dx() + x] *
                       Tensorize(gradientField(x, y, z));
            }
        }
    }

    cv::Mat matSum(sum);
    matSum /= v.dx() * v.dy() * v.dz();
    return StructureTensor{matSum};
}

EigenPairs Volume::eigenPairsAt(
    int x, int y, int z, int voxelRadius, int gradientKernelSize) const
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
        std::make_pair(eigenValues(0), e0),
        std::make_pair(eigenValues(1), e1),
        std::make_pair(eigenValues(2), e2),
    };
}

EigenPairs Volume::interpolatedEigenPairsAt(
    double x, double y, double z, int voxelRadius, int gradientKernelSize) const
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
        std::make_pair(eigenValues(0), e0),
        std::make_pair(eigenValues(1), e1),
        std::make_pair(eigenValues(2), e2),
    };
}

StructureTensor Tensorize(cv::Vec3d gradient)
{
    double ix = gradient(0);
    double iy = gradient(1);
    double iz = gradient(2);
    // clang-format off
    return StructureTensor{ix * ix, ix * iy, ix * iz,
                           ix * iy, iy * iy, iy * iz,
                           ix * iz, iy * iz, iz * iz};
    // clang-format on
}

Tensor3D<cv::Vec3d> Volume::volume_gradient_(
    const Tensor3D<double>& v, int gradientKernelSize) const
{
    // Limitation of OpenCV: Kernel size must be 1, 3, 5, or 7
    assert(
        (gradientKernelSize == 1 || gradientKernelSize == 3 ||
         gradientKernelSize == 5 || gradientKernelSize == 7) &&
        "gradientKernelSize must be one of [1, 3, 5, 7]");

    // Calculate gradient field around specified voxel
    Tensor3D<cv::Vec3d> gradientField{v.dx(), v.dy(), v.dz()};

    // First do XY gradients
    for (size_t z = 0; z < v.dz(); ++z) {
        auto xGradient = gradient_(v.xySlice(z), Axis::X, gradientKernelSize);
        auto yGradient = gradient_(v.xySlice(z), Axis::Y, gradientKernelSize);
        for (size_t y = 0; y < v.dy(); ++y) {
            for (size_t x = 0; x < v.dx(); ++x) {
                gradientField(x, y, z) = {xGradient(y, x), yGradient(y, x), 0};
            }
        }
    }

    // Then Z gradients
    for (size_t layer = 0; layer < v.dy(); ++layer) {
        auto zGradient =
            gradient_(v.xzSlice(layer), Axis::Y, gradientKernelSize);
        for (size_t z = 0; z < v.dz(); ++z) {
            for (size_t x = 0; x < v.dx(); ++x) {
                gradientField(x, layer, z)(2) = zGradient(z, x);
            }
        }
    }

    return gradientField;
}

// Helper function to calculate gradient using best choice for given kernel
// size. If the kernel size is 3, uses the Scharr() operator to calculate the
// gradient which is more accurate than 3x3 Sobel operator
cv::Mat_<double> Volume::gradient_(
    const cv::Mat_<double>& input, Axis axis, int ksize) const
{
    // OpenCV params for gradients
    // XXX Revisit this and see if changing these makes a big difference
    constexpr double SCALE = 1;
    constexpr double DELTA = 0;

    cv::Mat_<double> grad(input.rows, input.cols);

    switch (axis) {
        case Axis::X:
            if (ksize == 3) {
                cv::Scharr(
                    input, grad, CV_64F, 1, 0, SCALE, DELTA,
                    cv::BORDER_REPLICATE);
            } else {
                cv::Sobel(
                    input, grad, CV_64F, 1, 0, ksize, SCALE, DELTA,
                    cv::BORDER_REPLICATE);
            }
            break;
        case Axis::Y:
            if (ksize == 3) {
                cv::Scharr(
                    input, grad, CV_64F, 0, 1, SCALE, DELTA,
                    cv::BORDER_REPLICATE);
            } else {
                cv::Sobel(
                    input, grad, CV_64F, 0, 1, ksize, SCALE, DELTA,
                    cv::BORDER_REPLICATE);
            }
            break;
    }
    return grad;
}

std::unique_ptr<double[]> MakeUniformGaussianField(int radius)
{
    auto sideLength = 2 * static_cast<size_t>(radius) + 1;
    auto fieldSize = sideLength * sideLength * sideLength;
    auto field = std::unique_ptr<double[]>(new double[fieldSize]);
    double sum = 0;
    double sigma = 1;
    double sigma3 = sigma * sigma * sigma;
    double n = 1 / (sigma3 * std::pow(2 * M_PI, 3.0 / 2.0));

    // Fill field
    for (int z = -radius; z <= radius; ++z) {
        for (int y = -radius; y <= radius; ++y) {
            for (int x = -radius; x <= radius; ++x) {
                double val = std::exp(-(x * x + y * y + z * z));
                auto index = static_cast<size_t>(
                    (z + radius) * sideLength * sideLength +
                    (y + radius) * sideLength + (x + radius));
                field[index] = n * val;
                sum += val;
            }
        }
    }

    // Normalize
    for (size_t i = 0; i < sideLength * sideLength * sideLength; ++i) {
        field[i] /= sum;
    }

    return field;
}

Neighborhood Volume::getVoxelNeighborsLinearInterpolated(
    const cv::Vec3d& center,
    const cv::Vec3d& majorAxis,
    double radius,
    double interval,
    Direction direction) const
{
    // Interval bounds
    if (AlmostEqual(interval, 0.0)) {
        throw std::domain_error("Sampling interval too small");
    }

    // Normalize axis and ensure properly oriented radius
    cv::Vec3d axis;
    cv::normalize(majorAxis, axis);
    radius = std::abs(radius);

    // Setup Range
    double min, max;
    switch (direction) {
        case Direction::Bidirectional: {
            min = -1 * radius;
            max = radius;
            break;
        }
        case Direction::Positive: {
            min = 0;
            max = radius;
            break;
        }
        case Direction::Negative: {
            min = -1 * radius;
            max = 0;
            break;
        }
    }

    // Iterate through range
    Neighborhood n;
    auto count = static_cast<size_t>(std::floor((max - min) / interval) + 1);
    for (size_t it = 0; it < count; it++) {
        auto offset = min + (it * interval);
        n.emplace_back(interpolateAt(center + (axis * offset)));
    }

    return n;
}