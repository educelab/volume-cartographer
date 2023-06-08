#include "vc/core/types/Volume.hpp"

#include <iomanip>
#include <sstream>

#include <opencv2/imgcodecs.hpp>

#include "vc/core/io/TIFFIO.hpp"

namespace fs = volcart::filesystem;
namespace tio = volcart::tiffio;

using namespace volcart;

// Load a Volume from disk
Volume::Volume(fs::path path) : DiskBasedObjectBaseClass(std::move(path))
{
    if (metadata_.get<std::string>("type") != "vol") {
        throw std::runtime_error("File not of type: vol");
    }

    width_ = metadata_.get<int>("width");
    height_ = metadata_.get<int>("height");
    slices_ = metadata_.get<int>("slices");
    numSliceCharacters_ = std::to_string(slices_).size();

    std::vector<std::mutex> init_mutexes(slices_);

    slice_mutexes_.swap(init_mutexes);
}

// Setup a Volume from a folder of slices
Volume::Volume(fs::path path, std::string uuid, std::string name)
    : DiskBasedObjectBaseClass(
          std::move(path), std::move(uuid), std::move(name)),
          slice_mutexes_(slices_)
{
    metadata_.set("type", "vol");
    metadata_.set("width", width_);
    metadata_.set("height", height_);
    metadata_.set("slices", slices_);
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

int Volume::sliceWidth() const { return width_; }
int Volume::sliceHeight() const { return height_; }
int Volume::numSlices() const { return slices_; }
double Volume::voxelSize() const { return metadata_.get<double>("voxelsize"); }
double Volume::min() const { return metadata_.get<double>("min"); }
double Volume::max() const { return metadata_.get<double>("max"); }

void Volume::setSliceWidth(int w)
{
    width_ = w;
    metadata_.set("width", w);
}

void Volume::setSliceHeight(int h)
{
    height_ = h;
    metadata_.set("height", h);
}

void Volume::setNumberOfSlices(size_t numSlices)
{
    slices_ = numSlices;
    numSliceCharacters_ = std::to_string(numSlices).size();
    metadata_.set("slices", numSlices);
}

void Volume::setVoxelSize(double s) { metadata_.set("voxelsize", s); }
void Volume::setMin(double m) { metadata_.set("min", m); }
void Volume::setMax(double m) { metadata_.set("max", m); }

Volume::Bounds Volume::bounds() const
{
    return {
        {0, 0, 0},
        {static_cast<double>(width_), static_cast<double>(height_),
         static_cast<double>(slices_)}};
}

bool Volume::isInBounds(double x, double y, double z) const
{
    return x >= 0 && x < width_ && y >= 0 && y < height_ && z >= 0 &&
           z < slices_;
}

bool Volume::isInBounds(const cv::Vec3d& v) const
{
    return isInBounds(v(0), v(1), v(2));
}

fs::path Volume::getSlicePath(int index) const
{
    std::stringstream ss;
    ss << std::setw(numSliceCharacters_) << std::setfill('0') << index
       << ".tif";
    return path_ / ss.str();
}

cv::Mat Volume::getSliceData(int index) const
{
    if (cacheSlices_) {
        return cache_slice_(index);
    } else {
        return load_slice_(index);
    }
}

cv::Mat Volume::getSliceDataCopy(int index) const
{
    return getSliceData(index).clone();
}

cv::Mat Volume::getSliceDataRect(int index, cv::Rect rect) const
{
    auto whole_img = getSliceData(index);
    std::shared_lock<std::shared_mutex> lock(cache_mutex_);
    return whole_img(rect);
}

cv::Mat Volume::getSliceDataRectCopy(int index, cv::Rect rect) const
{
    auto whole_img = getSliceData(index);
    std::shared_lock<std::shared_mutex> lock(cache_mutex_);
    return whole_img(rect).clone();
}

void Volume::setSliceData(int index, const cv::Mat& slice, bool compress)
{
    auto slicePath = getSlicePath(index);
    tio::WriteTIFF(
        slicePath.string(), slice,
        (compress) ? tiffio::Compression::LZW : tiffio::Compression::NONE);
}

uint16_t Volume::intensityAt(int x, int y, int z) const
{
    // clang-format off
    if (x < 0 || x >= sliceWidth() ||
        y < 0 || y >= sliceHeight() ||
        z < 0 || z >= numSlices()) {
        return 0;
    }
    // clang-format on
    return getSliceData(z).at<uint16_t>(y, x);
}

// Trilinear Interpolation
// From: https://en.wikipedia.org/wiki/Trilinear_interpolation
uint16_t Volume::interpolateAt(double x, double y, double z) const
{
    // insert safety net
    if (!isInBounds(x, y, z)) {
        return 0;
    }

    double intPart;
    double dx = std::modf(x, &intPart);
    auto x0 = static_cast<int>(intPart);
    int x1 = x0 + 1;
    double dy = std::modf(y, &intPart);
    auto y0 = static_cast<int>(intPart);
    int y1 = y0 + 1;
    double dz = std::modf(z, &intPart);
    auto z0 = static_cast<int>(intPart);
    int z1 = z0 + 1;

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

Reslice Volume::reslice(
    const cv::Vec3d& center,
    const cv::Vec3d& xvec,
    const cv::Vec3d& yvec,
    int width,
    int height) const
{
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

cv::Mat Volume::load_slice_(int index) const
{
    std::cout << "Requested to load slice " << index << std::endl;
    auto slicePath = getSlicePath(index);
    return cv::imread(slicePath.string(), -1);
}

// cv::Mat Volume::cache_slice_(int index) const
// {
//     {
//         std::shared_lock<std::shared_mutex> lock(cache_mutex_);
//         if (cache_->contains(index)) {
//             return cache_->get(index);
//         }
//     }
//     {
//         auto slice = load_slice_(index);
//         std::unique_lock<std::shared_mutex> lock(cache_mutex_);
//         if (!cache_->contains(index)) {
//             cache_->put(index, slice);
//         }
//         return slice;
//     }
//     // {
//     //     std::shared_lock<std::shared_mutex> lock(cache_mutex_);
//     //     if (cache_->contains(index)) {
//     //         return cache_->get(index);
//     //     }
//     // }
//     // {
//     //     std::unique_lock<std::shared_mutex> lock(cache_mutex_);
//     //     auto slice = load_slice_(index);
//     //     if (!cache_->contains(index)) {
//     //         cache_->put(index, slice);
//     //     }
//     //     return slice;
//     // }
//     // {
//     //     std::shared_lock<std::shared_mutex> lock(cache_mutex_);
//     //     if (cache_->contains(index)) {
//     //         return cache_->get(index);
//     //     }
//     // }
//     // {
//     //     std::unique_lock<std::shared_mutex> lock(cache_mutex_);
//     //     if (cache_->contains(index)) {
//     //         return cache_->get(index);
//     //     }
//     //     else {
//     //         auto slice = load_slice_(index);
//     //         cache_->put(index, slice);
//     //         return slice;
//     //     }
//     // }
// }

cv::Mat Volume::cache_slice_(int index) const
{
    // Check if the slice is in the cache.
    {
        std::shared_lock<std::shared_mutex> lock(cache_mutex_);
        if (cache_->contains(index)) {
            return cache_->get(index);
        }
    }

    cv::Mat slice;

    {
        // Get the lock for this slice.
        auto& mutex = slice_mutexes_[index];

        // If the slice is not in the cache, get exclusive access to this slice's mutex.
        std::unique_lock<std::mutex> lock(mutex);
        // Check again to ensure the slice has not been added to the cache while waiting for the lock.
        {
            std::shared_lock<std::shared_mutex> lock(cache_mutex_);
            if (cache_->contains(index)) {
                return cache_->get(index);
            }
        }
        // Load the slice and add it to the cache.
        {
            std::unique_lock<std::shared_mutex> lock(cache_mutex_);
            slice = load_slice_(index);
            cache_->put(index, slice);
        }
    }

    return slice;
}


void Volume::cachePurge() const 
{
    std::unique_lock<std::shared_mutex> lock(cache_mutex_);
    cache_->purge();
}

