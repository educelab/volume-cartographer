#pragma once

#ifndef _VOLCART_VOLUME_H_
#define _VOLCART_VOLUME_H_

#include <string>
#include <array>
#include <opencv2/core/core.hpp>
#include <boost/filesystem.hpp>
#include "LRUCache.h"
#include "Tensor3D.h"
#include "Slice.h"

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

    Volume(boost::filesystem::path slicePath,
           boost::filesystem::path normalPath, int32_t nslices,
           int32_t sliceWidth, int32_t sliceHeight)
        : slicePath_(slicePath),
          normalPath_(normalPath),
          numSlices_(nslices),
          sliceWidth_(sliceWidth),
          sliceHeight_(sliceHeight)
    {
        numSliceCharacters_ = std::to_string(nslices).size();
    }

    // Returning const ref doesn't actually forbid modifying cv::Mat data by the
    // compiler, but it should serve as a warning to the programmer that you
    // shouldn't monkey around with it.
    const cv::Mat& getSliceData(const int32_t index) const;

    // Instead, supply this function that will return a copy of the data
    cv::Mat getSliceDataCopy(const int32_t index) const;

    bool setSliceData(const int32_t index, const cv::Mat& slice);

    boost::filesystem::path getSlicePath(const int32_t index) const;

    boost::filesystem::path getNormalPathAtIndex(const int32_t index) const;

    uint16_t interpolateAt(const Voxel point) const;

    uint16_t interpolatedIntensityAt(const Voxel nonGridPoint) const
    {
        return interpolateAt(nonGridPoint);
    }

    uint16_t interpolatedIntensityAt(const double x, const double y,
                                     const double z) const
    {
        return interpolateAt({x, y, z});
    }

    uint16_t intensityAt(const cv::Vec3d v) const
    {
        return intensityAt(int32_t(v(0)), int32_t(v(1)), int32_t(v(2)));
    }

    uint16_t intensityAt(const int32_t x, const int32_t y,
                         const int32_t z) const
    {
        return getSliceData(z).at<uint16_t>(y, x);
    }

    void setCacheSize(const size_t newCacheSize)
    {
        cache_.setSize(newCacheSize);
    }

    size_t getCacheSize() const { return cache_.size(); };
    void setCacheMemoryInBytes(const size_t nbytes)
    {
        setCacheSize(nbytes / (sliceWidth_ * sliceHeight_));
    }

    Slice reslice(const Voxel center, const cv::Vec3d xvec,
                  const cv::Vec3d yvec, const int32_t width = 64,
                  const int32_t height = 64) const;

    StructureTensor structureTensorAt(
        const int32_t x, const int32_t y, const int32_t z,
        const int32_t voxelRadius = 1,
        const int32_t gradientKernelSize = 3) const;

    StructureTensor structureTensorAt(
        const cv::Point3i index, const int32_t voxelRadius = 1,
        const int32_t gradientKernelSize = 3) const
    {
        return structureTensorAt(index.x, index.y, index.z, voxelRadius,
                                 gradientKernelSize);
    }

    StructureTensor interpolatedStructureTensorAt(
        const double x, const double y, const double z,
        const int32_t voxelRadius = 1,
        const int32_t gradientKernelSize = 3) const;

    StructureTensor interpolatedStructureTensorAt(
        const cv::Point3d index, const int32_t voxelRadius = 1,
        const int32_t gradientKernelSize = 3) const
    {
        return interpolatedStructureTensorAt(index.x, index.y, index.z,
                                             voxelRadius, gradientKernelSize);
    }

    EigenPairs eigenPairsAt(const int32_t x, const int32_t y, const int32_t z,
                            const int32_t voxelRadius = 1,
                            const int32_t gradientKernelSize = 3) const;

    EigenPairs eigenPairsAt(const cv::Point3i index,
                            const int32_t voxelRadius = 1,
                            const int32_t gradientKernelSize = 3) const
    {
        return eigenPairsAt(index.x, index.y, index.z, voxelRadius,
                            gradientKernelSize);
    }

    EigenPairs interpolatedEigenPairsAt(
        const double x, const double y, const double z,
        const int32_t voxelRadius = 1,
        const int32_t gradientKernelSize = 3) const;

    EigenPairs interpolatedEigenPairsAt(
        const cv::Point3d index, const int32_t voxelRadius = 1,
        const int32_t gradientKernelSize = 3) const
    {
        return eigenPairsAt(index.x, index.y, index.z, voxelRadius,
                            gradientKernelSize);
    }

    template <typename DType>
    Tensor3D<DType> getVoxelNeighbors(const cv::Point3i center,
                                      const int32_t rx, const int32_t ry,
                                      const int32_t rz) const
    {
        // Safety checks
        assert(center.x >= 0 && center.x < sliceWidth_ && center.y >= 0 &&
               center.y < sliceHeight_ && center.z >= 0 &&
               center.z < numSlices_ && "center must be inside volume");

        Tensor3D<DType> v(2 * rx + 1, 2 * ry + 1, 2 * rz + 1);
        for (int32_t k = center.z - rz, c = 0; k <= center.z + rz; ++k, ++c) {
            // If k index is out of bounds, then keep it at zeros and go on
            if (k < 0) {
                continue;
            }
            for (int32_t j = center.y - ry, b = 0; j <= center.y + ry;
                 ++j, ++b) {
                for (int32_t i = center.x - rx, a = 0; i <= center.x + rx;
                     ++i, ++a) {
                    if (i >= 0 && j >= 0) {
                        v(a, b, c) = DType(intensityAt(i, j, k));
                    }
                }
            }
        }

        return v;
    }

    template <typename DType>
    Tensor3D<DType> getVoxelNeighborsCubic(const cv::Point3i center,
                                           const int32_t radius) const
    {
        return getVoxelNeighbors<DType>(center, radius, radius, radius);
    }

    template <typename DType>
    Tensor3D<DType> getVoxelNeighborsInterpolated(const cv::Point3d center,
                                                  const int32_t rx,
                                                  const int32_t ry,
                                                  const int32_t rz) const
    {
        // Safety checks
        assert(center.x >= 0 && center.x < sliceWidth_ && center.y >= 0 &&
               center.y < sliceHeight_ && center.z >= 0 &&
               center.z < numSlices_ && "center must be inside volume");

        Tensor3D<DType> v(2 * rx + 1, 2 * ry + 1, 2 * rz + 1);
        double i, j, k;
        int32_t a, b, c;
        for (k = center.z - rz, c = 0; k <= center.z + rz; k += 1.0f, ++c) {
            // If k index is out of bounds, then keep it at zeros and go on
            if (k < 0) {
                continue;
            }
            for (j = center.y - ry, b = 0; j <= center.y + ry; j += 1.0f, ++b) {
                for (i = center.x - rx, a = 0; i <= center.x + rx;
                     i += 1.0f, ++a) {
                    if (i >= 0 && j >= 0) {
                        v(a, b, c) = DType(interpolatedIntensityAt(i, j, k));
                    }
                }
            }
        }

        return v;
    }

    template <typename DType>
    Tensor3D<DType> getVoxelNeighborsCubicInterpolated(
        const cv::Point3d center, const int32_t radius) const
    {
        return getVoxelNeighborsInterpolated<DType>(center, radius, radius,
                                                    radius);
    }

private:
    boost::filesystem::path slicePath_;
    boost::filesystem::path normalPath_;
    int32_t numSlices_;
    int32_t sliceWidth_;
    int32_t sliceHeight_;
    int32_t numSliceCharacters_;
    mutable volcart::LRUCache<int32_t, cv::Mat> cache_;

    Tensor3D<cv::Vec3d> volumeGradient(const Tensor3D<double>& v,
                                       const int32_t gradientKernelSize) const;

    cv::Mat_<double> gradient(const cv::Mat_<double>& input,
                              const GradientAxis axis,
                              const int32_t gradientKernelSize) const;
};
}

#endif
