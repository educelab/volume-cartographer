#include "Volume.h"
#include <sstream>
#include <memory>
#include <iomanip>

using namespace volcart;

const StructureTensor zeroStructureTensor =
    StructureTensor(0, 0, 0, 0, 0, 0, 0, 0, 0);

StructureTensor makeStructureTensor(const cv::Vec3d gradient);

std::unique_ptr<double[]> makeGaussianField(const int32_t radius);

// Trilinear Interpolation: Particles are not required
// to be at integer positions so we estimate their
// normals with their neighbors's known normals.
//
// formula from http://paulbourke.net/miscellaneous/interpolation/
uint16_t Volume::interpolateAt(const Voxel point) const
{
    double int_part;
    double dx = modf(point(0), &int_part);
    int x0 = int(int_part);
    int x1 = x0 + 1;
    double dy = modf(point(1), &int_part);
    int y0 = int(int_part);
    int y1 = y0 + 1;
    double dz = modf(point(2), &int_part);
    int z0 = int(int_part);
    int z1 = z0 + 1;

    // insert safety net
    if (x0 < 0 || y0 < 0 || z0 < 0 || x1 >= sliceWidth_ || y1 >= sliceHeight_ ||
        z1 >= ssize_t(numSlices_)) {
        return 0;
    }

    // from: https://en.wikipedia.org/wiki/Trilinear_interpolation
    auto c00 = getIntensityAtCoord(x0, y0, z0) * (1 - dx) +
               getIntensityAtCoord(x1, y0, z0) * dx;
    auto c10 = getIntensityAtCoord(x0, y1, z0) * (1 - dx) +
               getIntensityAtCoord(x1, y0, z0) * dx;
    auto c01 = getIntensityAtCoord(x0, y0, z1) * (1 - dx) +
               getIntensityAtCoord(x1, y0, z1) * dx;
    auto c11 = getIntensityAtCoord(x0, y1, z1) * (1 - dx) +
               getIntensityAtCoord(x1, y1, z1) * dx;

    auto c0 = c00 * (1 - dy) + c10 * dy;
    auto c1 = c01 * (1 - dy) + c11 * dy;

    auto c = c0 * (1 - dz) + c1 * dz;
    return uint16_t(cvRound(c));
}

cv::Mat Volume::getSliceData(const int32_t index) const
{
    // Take advantage of caching layer
    const auto possibleSlice = cache_.get(index);
    if (possibleSlice != nullptr) {
        return *possibleSlice;
    }

    const auto slicePath = getSlicePath(index);
    const cv::Mat sliceImg = cv::imread(slicePath.string(), -1);

    // Put into cache so we can use it later
    cache_.put(index, sliceImg);

    return sliceImg;
}

// Data Assignment
bool Volume::setSliceData(const int32_t index, const cv::Mat& slice)
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

boost::filesystem::path Volume::getNormalPathAtIndex(const int32_t index) const
{
    std::stringstream ss;
    ss << std::setw(numSliceCharacters_) << std::setfill('0') << index
       << ".pcd";
    return normalPath_ / ss.str();
}

boost::filesystem::path Volume::getSlicePath(const int32_t index) const
{
    std::stringstream ss;
    ss << std::setw(numSliceCharacters_) << std::setfill('0') << index
       << ".tif";
    return slicePath_ / ss.str();
}

Slice Volume::reslice(const Voxel center, const cv::Vec3d xvec,
                      const cv::Vec3d yvec, const int32_t width,
                      const int32_t height) const
{
    const auto xnorm = cv::normalize(xvec);
    const auto ynorm = cv::normalize(yvec);
    const auto origin = center - ((width / 2) * xnorm + (height / 2) * ynorm);

    cv::Mat m(height, width, CV_16UC1);
    for (int h = 0; h < height; ++h) {
        for (int w = 0; w < width; ++w) {
            auto val = interpolateAt(origin + (h * ynorm) + (w * xnorm));
            m.at<uint16_t>(h, w) = val;
        }
    }

    return Slice(m, origin, xnorm, ynorm);
}

StructureTensor Volume::structureTensorAtIndex(const int32_t vx,
                                               const int32_t vy,
                                               const int32_t vz,
                                               const int32_t voxelRadius) const
{
    using cv::Range;

    // Safety checks
    assert(vx >= 0 && vx < sliceWidth_ &&
           "x must be in range [0, slice_width]\n");
    assert(vy >= 0 && vy < sliceHeight_ &&
           "y must be in range [0, slice_height]\n");
    assert(vz >= 0 && vz < numSlices_ && "z must be in range [0, #slices]\n");

    // Get voxels in radius voxelRadius around voxel (x, y, z)
    auto v = getVoxelNeighborsCubic<double>({vx, vy, vz}, voxelRadius);

    // Calculate gradient field around specified voxel
    const int32_t n = 2 * voxelRadius + 1;
    auto gradientField = Tensor3D<cv::Vec3d>(n, n, n);

    // First do XY gradients
    for (int32_t z = 0; z < n; ++z) {
        cv::Mat_<double> xGradient(n, n);
        cv::Mat_<double> yGradient(n, n);
        cv::Scharr(v.xySlice(z), xGradient, CV_64F, 1, 0);
        cv::Scharr(v.xySlice(z), yGradient, CV_64F, 0, 1);
        for (int32_t y = 0; y < n; ++y) {
            for (int32_t x = 0; x < n; ++x) {
                gradientField(x, y, z) = {xGradient(y, x), yGradient(y, x), 0};
            }
        }
    }

    // Then Z gradients
    for (int32_t layer = 0; layer < n; ++layer) {
        cv::Mat_<double> zGradient(n, n);
        cv::Scharr(v.xzSlice(layer), zGradient, CV_64F, 0, 1);
        for (int32_t z = 0; z < n; ++z) {
            for (int32_t x = 0; x < n; ++x) {
                gradientField(x, layer, z)(2) = zGradient(z, x);
            }
        }
    }

    // Convert to tensor volume
    // XXX: Still doing Mike's algorithm for calculating. Assumes radius=1
    return (1.0 / 7.0) * (makeStructureTensor(gradientField(0, 1, 1)) +
                          makeStructureTensor(gradientField(1, 1, 1)) +
                          makeStructureTensor(gradientField(2, 1, 1)) +
                          makeStructureTensor(gradientField(1, 1, 0)) +
                          makeStructureTensor(gradientField(1, 1, 2)) +
                          makeStructureTensor(gradientField(1, 0, 1)) +
                          makeStructureTensor(gradientField(1, 2, 1)));

    // Make tensor field
    // Note: This can just be a 1-D array of StructureTensor since we no longer
    // care about
    // the ordering
    /*
    auto tensorField = std::vector<StructureTensor>();
    tensorField.reserve(n * n * n);
    for (int32_t z = 0; z < n; ++z) {
        for (int32_t y = 0; y < n; ++y) {
            for (int32_t x = 0; x < n; ++x) {
                tensorField.push_back(
                    makeStructureTensor(gradientField(x, y, z)));
            }
        }
    }

    // Modulate by gaussian distribution (element-wise) and sum
    // The ordeings must match - i.e. the order of adding the tensors to the
    // field must match that of the Gaussian weightings
    auto gaussianField = makeGaussianField(voxelRadius);
    StructureTensor sum = StructureTensor(0, 0, 0, 0, 0, 0, 0, 0, 0);
    for (size_t i = 0; i < size_t(n * n * n); ++i) {
        sum += tensorField[i] * gaussianField[i];
    }

    return sum;
    */
}

EigenPairs Volume::eigenPairsAtIndex(const int32_t x, const int32_t y,
                                     const int32_t z,
                                     const int32_t voxelRadius) const
{
    auto st = structureTensorAtIndex(x, y, z, voxelRadius);
    if (st == zeroStructureTensor) {
        throw ZeroStructureTensorException();
    }
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

StructureTensor makeStructureTensor(const cv::Vec3d gradient)
{
    double Ix = gradient(0);
    double Iy = gradient(1);
    double Iz = gradient(2);
    // clang-format off
    return StructureTensor(Ix * Ix, Ix * Iy, Ix * Iz,
                           Ix * Iy, Iy * Iy, Iy * Iz,
                           Ix * Iz, Iy * Iz, Iz * Iz);
    // clang-format on
}

std::unique_ptr<double[]> makeGaussianField(const int32_t radius)
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
