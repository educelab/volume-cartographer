#pragma once

#include <array>
#include <string>

#include <boost/filesystem.hpp>
#include <opencv2/core.hpp>

#include "core/types/LRUCache.h"
#include "core/types/Slice.h"
#include "core/types/Tensor3D.h"

using Voxel = cv::Vec3d;
using EigenValue = double;
using EigenVector = cv::Vec3d;
using EigenPairs = std::array<std::pair<EigenValue, EigenVector>, 3>;
using StructureTensor = cv::Matx33d;

namespace volcart
{
const static StructureTensor ZeroStructureTensor =
    StructureTensor(0, 0, 0, 0, 0, 0, 0, 0, 0);

class Volume
{
public:
    enum class GradientAxis { X, Y };

    Volume() = default;

    Volume(
        boost::filesystem::path slicePath,
        int32_t nslices,
        int32_t sliceWidth,
        int32_t sliceHeight)
        : slicePath_(slicePath)
        , numSlices_(nslices)
        , sliceWidth_(sliceWidth)
        , sliceHeight_(sliceHeight)
    {
        numSliceCharacters_ = std::to_string(nslices).size();
    }

    void setNumberOfSlices(size_t numSlices)
    {
        numSlices_ = numSlices;
        numSliceCharacters_ = std::to_string(numSlices_).size();
    }

    // Returning const ref doesn't actually forbid modifying cv::Mat data by the
    // compiler, but it should serve as a warning to the programmer that you
    // shouldn't monkey around with it.
    const cv::Mat& getSliceData(int32_t index) const;

    int32_t sliceWidth() const { return sliceWidth_; }

    int32_t sliceHeight() const { return sliceHeight_; }

    int32_t numSlices() const { return numSlices_; }

    bool isInBounds(const Voxel v) const
    {
        return v(0) >= 0 && v(0) < sliceWidth_ && v(1) >= 0 &&
               v(1) < sliceHeight_ && v(2) >= 0 && v(2) < numSlices_;
    }

    // Instead, supply this function that will return a copy of the data
    cv::Mat getSliceDataCopy(int32_t index) const;

    bool setSliceData(int32_t index, const cv::Mat& slice);

    boost::filesystem::path getSlicePath(int32_t index) const;

    boost::filesystem::path getNormalPathAtIndex(int32_t index) const;

    uint16_t interpolateAt(const Voxel point) const;

    uint16_t interpolatedIntensityAt(const Voxel nonGridPoint) const
    {
        return interpolateAt(nonGridPoint);
    }

    uint16_t interpolatedIntensityAt(double x, double y, double z) const
    {
        // clang-format off
        if (x < 0 || x >= sliceWidth_ ||
            y < 0 || y >= sliceHeight_ ||
            z < 0 || z >= numSlices_) {
            return 0;
        }
        // clang-format on
        return interpolateAt({x, y, z});
    }

    uint16_t intensityAt(const cv::Vec3d v) const
    {
        return intensityAt(int32_t(v(0)), int32_t(v(1)), int32_t(v(2)));
    }

    uint16_t intensityAt(
        const int32_t x, const int32_t y, const int32_t z) const
    {
        // clang-format off
        if (x < 0 || x >= sliceWidth_ ||
            y < 0 || y >= sliceHeight_ ||
            z < 0 || z >= numSlices_) {
            return 0;
        }
        // clang-format on
        return getSliceData(z).at<uint16_t>(y, x);
    }

    // Number of elements allowed in the cache
    void setCacheCapacity(size_t newCacheCapacity)
    {
        cache_.setCapacity(newCacheCapacity);
    }
    size_t getCacheCapacity() const { return cache_.capacity(); };

    void setCacheMemoryInBytes(size_t nbytes)
    {
        setCacheCapacity(nbytes / (sliceWidth_ * sliceHeight_));
    }

    // Number of elements in the cache
    size_t getCacheSize() const { return cache_.size(); };

    Slice reslice(
        const Voxel center,
        const cv::Vec3d xvec,
        const cv::Vec3d yvec,
        int32_t width = 64,
        int32_t height = 64) const;

    StructureTensor structureTensorAt(
        int32_t x,
        int32_t y,
        int32_t z,
        int32_t voxelRadius = 1,
        int32_t gradientKernelSize = 3) const;

    StructureTensor structureTensorAt(
        const cv::Point3i index,
        int32_t voxelRadius = 1,
        int32_t gradientKernelSize = 3) const
    {
        return structureTensorAt(
            index.x, index.y, index.z, voxelRadius, gradientKernelSize);
    }

    StructureTensor interpolatedStructureTensorAt(
        double x,
        double y,
        double z,
        int32_t voxelRadius = 1,
        int32_t gradientKernelSize = 3) const;

    StructureTensor interpolatedStructureTensorAt(
        const cv::Point3d index,
        int32_t voxelRadius = 1,
        int32_t gradientKernelSize = 3) const
    {
        return interpolatedStructureTensorAt(
            index.x, index.y, index.z, voxelRadius, gradientKernelSize);
    }

    EigenPairs eigenPairsAt(
        int32_t x,
        int32_t y,
        int32_t z,
        int32_t voxelRadius = 1,
        int32_t gradientKernelSize = 3) const;

    EigenPairs eigenPairsAt(
        const cv::Point3i index,
        int32_t voxelRadius = 1,
        int32_t gradientKernelSize = 3) const
    {
        return eigenPairsAt(
            index.x, index.y, index.z, voxelRadius, gradientKernelSize);
    }

    EigenPairs interpolatedEigenPairsAt(
        double x,
        double y,
        double z,
        int32_t voxelRadius = 1,
        int32_t gradientKernelSize = 3) const;

    EigenPairs interpolatedEigenPairsAt(
        const cv::Point3d index,
        int32_t voxelRadius = 1,
        int32_t gradientKernelSize = 3) const
    {
        return interpolatedEigenPairsAt(
            index.x, index.y, index.z, voxelRadius, gradientKernelSize);
    }

    template <typename DType>
    Tensor3D<DType> getVoxelNeighbors(
        const cv::Point3i center, int32_t rx, int32_t ry, int32_t rz) const
    {
        // Safety checks
        assert(
            center.x >= 0 && center.x < sliceWidth_ && center.y >= 0 &&
            center.y < sliceHeight_ && center.z >= 0 && center.z < numSlices_ &&
            "center must be inside volume");

        Tensor3D<DType> v(2 * rx + 1, 2 * ry + 1, 2 * rz + 1);
        for (int32_t k = center.z - rz, c = 0; k <= center.z + rz; ++k, ++c) {
            // If k index is out of bounds, then keep it at zeros and go on
            if (k < 0 || k >= numSlices_) {
                continue;
            }
            for (int32_t j = center.y - ry, b = 0; j <= center.y + ry;
                 ++j, ++b) {
                for (int32_t i = center.x - rx, a = 0; i <= center.x + rx;
                     ++i, ++a) {
                    if (i >= 0 && j >= 0 && i < sliceWidth_ &&
                        j < sliceHeight_) {
                        v(a, b, c) = DType(intensityAt(i, j, k));
                    }
                }
            }
        }

        return v;
    }

    template <typename DType>
    Tensor3D<DType> getVoxelNeighborsCubic(
        const cv::Point3i center, int32_t radius) const
    {
        return getVoxelNeighbors<DType>(center, radius, radius, radius);
    }

    template <typename DType>
    Tensor3D<DType> getVoxelNeighborsInterpolated(
        const cv::Point3d center, int32_t rx, int32_t ry, int32_t rz) const
    {
        // Safety checks
        assert(
            center.x >= 0 && center.x < sliceWidth_ && center.y >= 0 &&
            center.y < sliceHeight_ && center.z >= 0 && center.z < numSlices_ &&
            "center must be inside volume");

        Tensor3D<DType> v(2 * rx + 1, 2 * ry + 1, 2 * rz + 1);
        double i, j, k;
        int32_t a, b, c;
        for (k = center.z - rz, c = 0; k <= center.z + rz; k += 1.0f, ++c) {
            // If k index is out of bounds, then keep it at zeros and go on
            if (k < 0 || k >= numSlices_) {
                continue;
            }
            for (j = center.y - ry, b = 0; j <= center.y + ry; j += 1.0f, ++b) {
                for (i = center.x - rx, a = 0; i <= center.x + rx;
                     i += 1.0f, ++a) {
                    if (i >= 0 && j >= 0 && i < sliceWidth_ &&
                        j < sliceHeight_) {
                        v(a, b, c) = DType(interpolatedIntensityAt(i, j, k));
                    }
                }
            }
        }

        return v;
    }

    template <typename DType>
    Tensor3D<DType> getVoxelNeighborsCubicInterpolated(
        const cv::Point3d center, int32_t radius) const
    {
        return getVoxelNeighborsInterpolated<DType>(
            center, radius, radius, radius);
    }

private:
    boost::filesystem::path slicePath_;
    int32_t numSlices_;
    int32_t sliceWidth_;
    int32_t sliceHeight_;
    int32_t numSliceCharacters_;
    mutable volcart::LRUCache<int32_t, cv::Mat> cache_;

    Tensor3D<cv::Vec3d> volumeGradient(
        const Tensor3D<double>& v, int32_t gradientKernelSize) const;

    cv::Mat_<double> gradient(
        const cv::Mat_<double>& input,
        GradientAxis axis,
        int32_t gradientKernelSize) const;
};
}
