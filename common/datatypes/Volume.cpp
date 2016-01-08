#include "Volume.h"
#include <sstream>
#include <iomanip>

using namespace volcart;

// Trilinear Interpolation: Particles are not required
// to be at integer positions so we estimate their
// normals with their neighbors's known normals.
//
// formula from http://paulbourke.net/miscellaneous/interpolation/
uint16_t Volume::interpolateAt(const cv::Vec3f point) const
{
    double int_part;
    double dx = modf(point(VC_INDEX_X), &int_part);
    int x0 = int(int_part);
    int x1 = x0 + 1;
    double dy = modf(point(VC_INDEX_Y), &int_part);
    int y0 = int(int_part);
    int y1 = y0 + 1;
    double dz = modf(point(VC_INDEX_Z), &int_part);
    int z0 = int(int_part);
    int z1 = z0 + 1;

    // insert safety net
    if (x0 < 0 || y0 < 0 || z0 < 0 ||
        x1 >= sliceWidth_ || y1 >= sliceHeight_ || z1 >= ssize_t(numSlices_)) {
        return 0;
    }

    // from: https://en.wikipedia.org/wiki/Trilinear_interpolation
    auto c00 = getIntensityAtCoord(x0, y0, z0) * (1 - dx) + getIntensityAtCoord(x1, y0, z0) * dx;
    auto c10 = getIntensityAtCoord(x0, y1, z0) * (1 - dx) + getIntensityAtCoord(x1, y0, z0) * dx;
    auto c01 = getIntensityAtCoord(x0, y0, z1) * (1 - dx) + getIntensityAtCoord(x1, y0, z1) * dx;
    auto c11 = getIntensityAtCoord(x0, y1, z1) * (1 - dx) + getIntensityAtCoord(x1, y1, z1) * dx;

    auto c0 = c00 * (1 - dy) + c10 * dy;
    auto c1 = c01 * (1 - dy) + c11 * dy;

    auto c = c0 * (1 - dz) + c1 * dz;
    return uint16_t(cvRound(c));
}

cv::Mat Volume::getSliceData(const size_t index) const
{
    // Take advantage of caching layer
    auto possibleSlice = cache_.get(index);
    if (possibleSlice != nullptr) {
        return *possibleSlice;
    }

    cv::Mat sliceImg = cv::imread( getSlicePath(index), -1 );
    
    // Put into cache so we can use it later
    cache_.put(index, sliceImg);
    
    return sliceImg;
}

// Data Assignment
bool Volume::setSliceData(const size_t index, const cv::Mat& slice)
{
    if (_readOnly) {
        VC_ERR_READONLY();
    } else if ( index >= numSlices_ ) {
        std::cerr << "ERROR: Atttempted to save a slice image to an out of bounds index."
                  << std::endl;
        return false;
    }

    std::string filepath = getSlicePath(index);
    cv::imwrite(filepath, slice);
    return true;
}

std::string Volume::getSlicePath(const size_t index) const
{
    std::string path = slicePath_.string();
    std::string indexStr = std::to_string(index);
    std::stringstream ss;
    ss << std::setw(numSliceCharacters_) << std::setfill('0') << index;
    return path + ss.str() + ".tif";
}

uint16_t Volume::getIntensityAtCoord(const uint32_t x, const uint32_t y, const uint32_t z) const
{
    auto slice = getSliceData(z);
    return slice.at<uint16_t>(y, x);
}

void Volume::setCacheSize(const size_t newCacheSize)
{
    cache_.setSize(newCacheSize);
}

void Volume::setCacheMemoryInBytes(const size_t nbytes)
{
    size_t sliceSize = getSliceData(0).step[0] * getSliceData(0).rows;
    setCacheSize(nbytes/sliceSize);
}

StructureTensor Volume::getStructureTensor(const Voxel v) const;

StructureTensor Volume::getStructureTensor(const uint32_t x, const uint32_t y, const uint32_t z) const;

