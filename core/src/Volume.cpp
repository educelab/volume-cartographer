#include "vc/core/types/Volume.hpp"

#include <iomanip>
#include <sstream>

#include <opencv2/imgcodecs.hpp>

#include "vc/core/io/TIFFIO.hpp"
#include "vc/core/util/Logging.hpp"

namespace fs = volcart::filesystem;
namespace tio = volcart::tiffio;

using namespace volcart;

namespace
{
auto OnEject(int& key, Volume::SliceItem& value) -> bool
{
    auto& [img, mmapInfo] = value;

    // Can't eject if there are still references
    if (img.u and img.u->refcount > 0) {
        Logger()->trace("Slice {} still has references", key);
        return false;
    }
    // Explicitly unmap the file
    Logger()->trace("Unmapping slice {}", key);
    if (mmapInfo.has_value() and mmapInfo.value()) {
        UnmapFile(mmapInfo.value());
    }
    return true;
}
}  // namespace

// Load a Volume from disk
Volume::Volume(fs::path path) : DiskBasedObjectBaseClass(std::move(path))
{
    if (metadata_.get<std::string>("type") != "vol") {
        throw std::runtime_error("File not of type: vol");
    }

    width_ = metadata_.get<int>("width").value();
    height_ = metadata_.get<int>("height").value();
    slices_ = metadata_.get<int>("slices").value();
    numSliceCharacters_ = static_cast<int>(std::to_string(slices_).size());
    sliceMutexes_ = std::vector<std::mutex>(slices_);
    cache_->onEject(OnEject);
}

// Set up a Volume from a folder of slices
Volume::Volume(fs::path path, std::string uuid, std::string name)
    : DiskBasedObjectBaseClass(
          std::move(path), std::move(uuid), std::move(name))
{
    metadata_.set("type", "vol");
    metadata_.set("width", width_);
    metadata_.set("height", height_);
    metadata_.set("slices", slices_);
    metadata_.set("voxelsize", double{});
    metadata_.set("min", double{});
    metadata_.set("max", double{});
    cache_->onEject(OnEject);
}

// Load a Volume from disk, return a pointer
auto Volume::New(const fs::path& path) -> Pointer
{
    return std::make_shared<Volume>(path);
}

// Set a Volume from a folder of slices, return a pointer
auto Volume::New(
    const fs::path& path,
    const std::string& uuid,
    const std::string& name) -> Pointer
{
    return std::make_shared<Volume>(path, uuid, name);
}

auto Volume::sliceWidth() const -> int { return width_; }
auto Volume::sliceHeight() const -> int { return height_; }
auto Volume::numSlices() const -> int { return slices_; }
auto Volume::voxelSize() const -> double
{
    return metadata_.get<double>("voxelsize").value();
}
auto Volume::min() const -> double
{
    return metadata_.get<double>("min").value();
}
auto Volume::max() const -> double
{
    return metadata_.get<double>("max").value();
}

void Volume::setSliceWidth(const int w)
{
    width_ = w;
    metadata_.set("width", w);
}

void Volume::setSliceHeight(const int h)
{
    height_ = h;
    metadata_.set("height", h);
}

void Volume::setNumberOfSlices(const std::size_t numSlices)
{
    slices_ = static_cast<int>(numSlices);
    numSliceCharacters_ = static_cast<int>(std::to_string(numSlices).size());
    metadata_.set("slices", numSlices);
    sliceMutexes_ = std::vector<std::mutex>(slices_);
}

void Volume::setVoxelSize(const double s) { metadata_.set("voxelsize", s); }
void Volume::setMin(const double m) { metadata_.set("min", m); }
void Volume::setMax(const double m) { metadata_.set("max", m); }

auto Volume::bounds() const -> Volume::Bounds
{
    return {
        {0, 0, 0},
        {static_cast<double>(width_), static_cast<double>(height_),
         static_cast<double>(slices_)}};
}

auto Volume::isInBounds(const double x, const double y, const double z) const
    -> bool
{
    return x >= 0 && x < width_ && y >= 0 && y < height_ && z >= 0 &&
           z < slices_;
}

auto Volume::isInBounds(const cv::Vec3d& v) const -> bool
{
    return isInBounds(v(0), v(1), v(2));
}

void Volume::setMemoryMapSlices(const bool b) { memmap_ = b; }

auto Volume::getSlicePath(const int index) const -> fs::path
{
    std::stringstream ss;
    ss << std::setw(numSliceCharacters_) << std::setfill('0') << index
       << ".tif";
    return path_ / ss.str();
}

auto Volume::getSliceData(const int index) const -> cv::Mat
{
    if (cacheSlices_) {
        return cache_slice_(index);
    }
    // Never memory map if caching is disabled. This is mostly because there's
    // currently not a good way to clean up the mmap_info when not caching.
    return load_slice_(index);
}

auto Volume::getSliceDataCopy(const int index) const -> cv::Mat
{
    return getSliceData(index).clone();
}

auto Volume::getSliceDataRect(const int index, const cv::Rect rect) const
    -> cv::Mat
{
    const auto whole_img = getSliceData(index);
    std::shared_lock lock(cacheMutex_);
    return whole_img(rect);
}

auto Volume::getSliceDataRectCopy(const int index, const cv::Rect rect) const
    -> cv::Mat
{
    const auto whole_img = getSliceData(index);
    std::shared_lock lock(cacheMutex_);
    return whole_img(rect).clone();
}

void Volume::setSliceData(
    const int index, const cv::Mat& slice, const bool compress) const
{
    const auto slicePath = getSlicePath(index);
    tio::WriteTIFF(
        slicePath.string(), slice,
        compress ? tiffio::Compression::LZW : tiffio::Compression::NONE);
}

auto Volume::intensityAt(const int x, const int y, const int z) const
    -> std::uint16_t
{
    // clang-format off
    if (x < 0 || x >= sliceWidth() ||
        y < 0 || y >= sliceHeight() ||
        z < 0 || z >= numSlices()) {
        return 0;
    }
    // clang-format on
    return getSliceData(z).at<std::uint16_t>(y, x);
}

auto Volume::intensityAt(const cv::Vec3d& v) const -> std::uint16_t
{
    return intensityAt(static_cast<int>(v[0]), static_cast<int>(v[1]), static_cast<int>(v[2]));
}

// Trilinear Interpolation
// From: https://en.wikipedia.org/wiki/Trilinear_interpolation
auto Volume::interpolateAt(const double x, const double y, const double z) const
    -> std::uint16_t
{
    // insert safety net
    if (!isInBounds(x, y, z)) {
        return 0;
    }

    double intPart;
    const double dx = std::modf(x, &intPart);
    const auto x0 = static_cast<int>(intPart);
    const int x1 = x0 + 1;
    const double dy = std::modf(y, &intPart);
    const auto y0 = static_cast<int>(intPart);
    const int y1 = y0 + 1;
    const double dz = std::modf(z, &intPart);
    const auto z0 = static_cast<int>(intPart);
    const int z1 = z0 + 1;

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
    return static_cast<std::uint16_t>(cvRound(c));
}

auto Volume::interpolateAt(const cv::Vec3d& v) const -> std::uint16_t
{
    return interpolateAt(v[0], v[1], v[2]);
}

auto Volume::reslice(
    const cv::Vec3d& center,
    const cv::Vec3d& xvec,
    const cv::Vec3d& yvec,
    const int width,
    const int height) const -> Reslice
{
    auto xnorm = cv::normalize(xvec);
    auto ynorm = cv::normalize(yvec);
    auto origin = center - ((width / 2) * xnorm + (height / 2) * ynorm);

    cv::Mat m(height, width, CV_16UC1);
    for (int h = 0; h < height; ++h) {
        for (int w = 0; w < width; ++w) {
            m.at<std::uint16_t>(h, w) =
                interpolateAt(origin + (h * ynorm) + (w * xnorm));
        }
    }

    return {m, origin, xnorm, ynorm};
}
void Volume::setCacheSlices(const bool b) { cacheSlices_ = b; }

void Volume::setCache(SliceCache::Pointer c) const
{
    std::unique_lock lock(cacheMutex_);
    cache_ = std::move(c);
    cache_->onEject(OnEject);
}

void Volume::setCacheCapacity(const std::size_t newCacheCapacity) const
{
    std::unique_lock lock(cacheMutex_);
    cache_->setCapacity(newCacheCapacity);
}

void Volume::setCacheMemoryInBytes(const std::size_t nbytes) const
{
    // x2 because pixels are 16 bits normally. Not a great solution.
    setCacheCapacity(nbytes / (sliceWidth() * sliceHeight() * 2));
}

auto Volume::getCacheCapacity() const -> std::size_t
{
    std::shared_lock lock(cacheMutex_);
    return cache_->capacity();
}

auto Volume::getCacheSize() const -> std::size_t { return cache_->size(); }

auto Volume::load_slice_(int index, mmap_info* mmap_info) const -> cv::Mat
{
    Logger()->info("Requested load slice: {}", index);
    const auto slicePath = getSlicePath(index);
    cv::Mat mat;
    try {
        mat = tio::ReadTIFF(slicePath.string(), mmap_info);
    } catch (const std::runtime_error& e) {
        Logger()->warn("Failed to load slice {}: {}", index, e.what());
    }
    return mat;
}

auto Volume::cache_slice_(const int index) const -> cv::Mat
{
    // Check if the slice is in the cache.
    {
        std::shared_lock lock(cacheMutex_);
        if (cache_->contains(index)) {
            return cache_->get(index).first;
        }
    }

    {
        // If the slice is not in the cache, get exclusive access to this
        // slice's mutex. This slice can't be set until we're done.
        // TODO: Is this faster than just getting an unique cache lock?
        auto& mutex = sliceMutexes_[index];
        std::unique_lock lockSlice(mutex);

        // Check again to ensure the slice has not been added to the cache while
        // waiting for the lock.
        {
            std::shared_lock lockCache(cacheMutex_);
            if (cache_->contains(index)) {
                return cache_->get(index).first;
            }
        }

        // Load the slice and put it in the cache
        cv::Mat slice;
        std::optional<mmap_info> mmapInfo;
        // If memory mapping, get the mmap_info too
        if (memmap_) {
            mmap_info i;
            slice = load_slice_(index, &i);
            mmapInfo = i;
        } else {
            slice = load_slice_(index);
        }
        std::unique_lock lockCache(cacheMutex_);
        cache_->put(index, {slice, mmapInfo});
        return slice;
    }
}

void Volume::cachePurge() const
{
    std::unique_lock lock(cacheMutex_);
    cache_->purge();
}
