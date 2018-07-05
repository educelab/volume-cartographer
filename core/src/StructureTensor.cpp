#include "vc/core/math/StructureTensor.hpp"

#include <opencv2/imgproc.hpp>

using namespace volcart;

/** @enum Axis labels */
enum class Axis { X, Y };

StructureTensor Tensorize(cv::Vec3d gradient);
Tensor3D<cv::Vec3d> VolumeGradient(const Tensor3D<double>& v, int kernelSize);
cv::Mat_<double> Gradient(const cv::Mat_<double>& input, Axis axis, int ksize);
std::unique_ptr<double[]> MakeUniformGaussianField(int radius);

StructureTensor volcart::ComputeVoxelStructureTensor(
    const Volume::Pointer& volume,
    int vx,
    int vy,
    int vz,
    int radius,
    int kernelSize)
{
    if (kernelSize < 3) {
        throw std::invalid_argument("gradient kernel size must be at least 3");
    }

    // Get voxels in radius radius around voxel (x, y, z)
    auto v = ComputeVoxelNeighbors<double>(
        volume, {vx, vy, vz}, radius, radius, radius);

    // Normalize voxel neighbors to [0, 1]
    for (size_t z = 0; z < v.dz(); ++z) {
        v.xySlice(z) /= std::numeric_limits<uint16_t>::max();
    }

    // Get gradient of volume
    auto gradientField = VolumeGradient(v, kernelSize);

    // Modulate by gaussian distribution (element-wise) and sum
    auto gaussianField = MakeUniformGaussianField(radius);
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

StructureTensor volcart::ComputeVoxelStructureTensor(
    const Volume::Pointer& volume,
    const cv::Vec3i& index,
    int radius,
    int kernelSize)
{
    return ComputeVoxelStructureTensor(
        volume, index(0), index(1), index(2), radius, kernelSize);
}

StructureTensor volcart::ComputeSubvoxelStructureTensor(
    const Volume::Pointer& volume,
    double vx,
    double vy,
    double vz,
    int radius,
    int kernelSize)
{
    if (kernelSize < 3) {
        throw std::invalid_argument("gradient kernel size must be at least 3");
    }

    // Get voxels in radius radius around voxel (x, y, z)
    auto v = ComputeSubvoxelNeighbors<double>(
        volume, {vx, vy, vz}, radius, radius, radius);

    // Get gradient of volume
    auto gradientField = VolumeGradient(v, kernelSize);

    // Modulate by gaussian distribution (element-wise) and sum
    auto gaussianField = MakeUniformGaussianField(radius);
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

StructureTensor volcart::ComputeSubvoxelStructureTensor(
    const Volume::Pointer& volume,
    const cv::Vec3d& index,
    int radius,
    int kernelSize)
{
    return ComputeSubvoxelStructureTensor(
        volume, index(0), index(1), index(2), radius, kernelSize);
}

EigenPairs volcart::ComputeVoxelEigenPairs(
    const Volume::Pointer& volume,
    int x,
    int y,
    int z,
    int radius,
    int kernelSize)
{
    auto st = ComputeVoxelStructureTensor(volume, x, y, z, radius, kernelSize);
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

EigenPairs volcart::ComputeVoxelEigenPairs(
    const Volume::Pointer& volume,
    const cv::Vec3i& index,
    int radius,
    int kernelSize)
{
    return ComputeVoxelEigenPairs(
        volume, index(0), index(1), index(2), radius, kernelSize);
}

EigenPairs volcart::ComputeSubvoxelEigenPairs(
    const Volume::Pointer& volume,
    double x,
    double y,
    double z,
    int radius,
    int kernelSize)
{
    auto st =
        ComputeSubvoxelStructureTensor(volume, x, y, z, radius, kernelSize);
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

EigenPairs volcart::ComputeSubvoxelEigenPairs(
    const Volume::Pointer& volume,
    const cv::Vec3d& index,
    int radius,
    int kernelSize)
{
    return ComputeSubvoxelEigenPairs(
        volume, index(0), index(1), index(2), radius, kernelSize);
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

Tensor3D<cv::Vec3d> VolumeGradient(const Tensor3D<double>& v, int kernelSize)
{
    // Limitation of OpenCV: Kernel size must be 1, 3, 5, or 7
    assert(
        (kernelSize == 1 || kernelSize == 3 || kernelSize == 5 ||
         kernelSize == 7) &&
        "kernelSize must be one of [1, 3, 5, 7]");

    // Calculate gradient field around specified voxel
    Tensor3D<cv::Vec3d> gradientField{v.dx(), v.dy(), v.dz()};

    // First do XY gradients
    for (size_t z = 0; z < v.dz(); ++z) {
        auto xGradient = Gradient(v.xySlice(z), Axis::X, kernelSize);
        auto yGradient = Gradient(v.xySlice(z), Axis::Y, kernelSize);
        for (size_t y = 0; y < v.dy(); ++y) {
            for (size_t x = 0; x < v.dx(); ++x) {
                gradientField(x, y, z) = {xGradient(y, x), yGradient(y, x), 0};
            }
        }
    }

    // Then Z gradients
    for (size_t layer = 0; layer < v.dy(); ++layer) {
        auto zGradient = Gradient(v.xzSlice(layer), Axis::Y, kernelSize);
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
cv::Mat_<double> Gradient(const cv::Mat_<double>& input, Axis axis, int ksize)
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