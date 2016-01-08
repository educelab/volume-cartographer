#pragma once

#ifndef _VOLCART_VOLUME_H_
#define _VOLCART_VOLUME_H_

#include <string>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include "vc_defines.h"
#include "vc_datatypes.h"

namespace volcart {

class Volume
{
private:
    boost::filesystem::path slicePath_;
    size_t numSlices_;
    int32_t sliceWidth_;
    int32_t sliceHeight_;
    int32_t numSliceCharacters_;
    mutable volcart::LRUCache<int32_t, cv::Mat> cache_;

    uint16_t interpolateAt(const cv::Vec3f point) const;

public:
    Volume() = default;

    Volume(std::string slicePath, size_t nslices, int32_t sliceWidth, int32_t sliceHeight) :
        slicePath_(slicePath), numSlices_(nslices), sliceWidth_(sliceWidth), sliceHeight_(sliceHeight)
    {
        numSliceCharacters_ = std::to_string(nslices).size();
    }

    cv::Mat getSliceData(const size_t index) const;

    bool setSliceData(const size_t index, const cv::Mat& slice);

    std::string getSlicePath(const size_t index) const;

    std::string getNormalAtIndex(const size_t index) const;

    uint16_t getInterpolatedIntensity(const cv::Vec3d point) const
    {
        return interpolateAt(point);
    }

    uint16_t getIntensityAtCoord(const uint32_t x, const uint32_t y, const uint32_t z) const;

    void setCacheSize(const size_t newCacheSize);

    size_t getCacheSize() const { return cache_.size(); };

    void setCacheMemoryInBytes(const size_t nbytes);

    StructureTensor getStructureTensor(const Voxel v) const;

    StructureTensor getStructureTensor(const uint32_t x, const uint32_t y, const uint32_t z) const;
};

}

#endif
