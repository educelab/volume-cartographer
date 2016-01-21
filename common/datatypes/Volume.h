#pragma once

#ifndef _VOLCART_VOLUME_H_
#define _VOLCART_VOLUME_H_

#include <string>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include "LRUCache.h"
#include "Tensor3D.h"
#include "Slice.h"

#define VC_INDEX_X 0
#define VC_INDEX_Y 1
#define VC_INDEX_Z 2

using Voxel = cv::Vec3d;
using StructureTensor = cv::Matx33d;

namespace volcart
{

class Volume
{
private:
    boost::filesystem::path slicePath_;
    boost::filesystem::path normalPath_;
    int32_t numSlices_;
    int32_t sliceWidth_;
    int32_t sliceHeight_;
    int32_t numSliceCharacters_;
    mutable volcart::LRUCache<int32_t, cv::Mat> cache_;

    uint16_t interpolateAt(const Voxel point) const;

public:

    Volume() = default;

    Volume(boost::filesystem::path slicePath, boost::filesystem::path normalPath, int32_t nslices, int32_t sliceWidth,
            int32_t sliceHeight)
        : slicePath_(slicePath),
          normalPath_(normalPath),
          numSlices_(nslices),
          sliceWidth_(sliceWidth),
          sliceHeight_(sliceHeight)
    {
        numSliceCharacters_ = std::to_string(nslices).size();
    }

    cv::Mat getSliceData(const int32_t index) const;

    bool setSliceData(const int32_t index, const cv::Mat& slice);

    boost::filesystem::path getSlicePath(const int32_t index) const;

    boost::filesystem::path getNormalPathAtIndex(const int32_t index) const;

    uint16_t getInterpolatedIntensity(const Voxel nonGridPoint) const
    {
        return interpolateAt(nonGridPoint);
    }

    uint16_t getIntensityAtCoord(const cv::Vec3d v) const
    {
        return getIntensityAtCoord(v(0), v(1), v(2));
    }

    uint16_t getIntensityAtCoord(const int32_t x, const int32_t y,
                                 const int32_t z) const;

    void setCacheSize(const size_t newCacheSize);

    size_t getCacheSize() const { return cache_.size(); };

    void setCacheMemoryInBytes(const size_t nbytes);

    Slice reslice(const Voxel center, const cv::Vec3d xvec,
                  const cv::Vec3d yvec, const int32_t width = 64,
                  const int32_t height = 64) const;

    StructureTensor getStructureTensor(const int32_t x, const int32_t y,
                                       const int32_t z,
                                       const int32_t voxelRadius = 1) const;

    template <typename DType>
    Tensor3D<DType> getVoxelNeighbors(const cv::Point3i center, const int32_t dx, const int32_t dy, const int32_t dz) const;

    template <typename DType>
    Tensor3D<DType> getVoxelNeighborsCubic(const cv::Point3i origin,
                                           const int32_t diameter) const;
};
}

#endif
