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

    width_ = metadata_.get<int>("width").value();
    height_ = metadata_.get<int>("height").value();
    slices_ = metadata_.get<int>("slices").value();
    numSliceCharacters_ = static_cast<int>(std::to_string(slices_).size());
}

// Setup a Volume from a folder of slices
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
}

// Load a Volume from disk, return a pointer
auto Volume::New(fs::path path) -> Volume::Pointer
{
    return std::make_shared<Volume>(path);
}

// Set a Volume from a folder of slices, return a pointer
auto Volume::New(fs::path path, std::string uuid, std::string name)
    -> Volume::Pointer
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

void Volume::setNumberOfSlices(std::size_t numSlices)
{
    slices_ = static_cast<int>(numSlices);
    numSliceCharacters_ = static_cast<int>(std::to_string(numSlices).size());
    metadata_.set("slices", numSlices);
}

void Volume::setVoxelSize(double s) { metadata_.set("voxelsize", s); }
void Volume::setMin(double m) { metadata_.set("min", m); }
void Volume::setMax(double m) { metadata_.set("max", m); }

auto Volume::bounds() const -> Volume::Bounds
{
    return {
        {0, 0, 0},
        {static_cast<double>(width_), static_cast<double>(height_),
         static_cast<double>(slices_)}};
}

auto Volume::isInBounds(double x, double y, double z) const -> bool
{
    return x >= 0 && x < width_ && y >= 0 && y < height_ && z >= 0 &&
           z < slices_;
}

auto Volume::isInBounds(const cv::Vec3d& v) const -> bool
{
    return isInBounds(v(0), v(1), v(2));
}

auto Volume::getSlicePath(int index) const -> fs::path
{
    std::stringstream ss;
    ss << std::setw(numSliceCharacters_) << std::setfill('0') << index
       << ".tif";
    return path_ / ss.str();
}

auto Volume::getSliceData(int index) const -> cv::Mat
{
    if (cacheSlices_) {
        return cache_slice_(index);
    }
    return load_slice_(index);
}

auto Volume::getSliceDataCopy(int index) const -> cv::Mat
{
    return getSliceData(index).clone();
}

void Volume::setSliceData(int index, const cv::Mat& slice, bool compress)
{
    auto slicePath = getSlicePath(index);
    tio::WriteTIFF(
        slicePath.string(), slice,
        (compress) ? tiffio::Compression::LZW : tiffio::Compression::NONE);
}

auto Volume::intensityAt(int x, int y, int z) const -> std::uint16_t
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

// Trilinear Interpolation
// From: https://en.wikipedia.org/wiki/Trilinear_interpolation
auto Volume::interpolateAt(double x, double y, double z) const -> std::uint16_t
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
    return static_cast<std::uint16_t>(cvRound(c));
}

auto Volume::reslice(
    const cv::Vec3d& center,
    const cv::Vec3d& xvec,
    const cv::Vec3d& yvec,
    int width,
    int height) const -> Reslice
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

    return Reslice(m, origin, xnorm, ynorm);
}

auto Volume::load_slice_(int index) const -> cv::Mat
{
    auto slicePath = getSlicePath(index);
    return cv::imread(slicePath.string(), -1);
}

auto Volume::cache_slice_(int index) const -> cv::Mat
{
    const std::lock_guard<std::mutex> lock(cacheMutex_);
    if (cache_->contains(index)) {
        return cache_->get(index);
    }

    auto slice = load_slice_(index);
    cache_->put(index, slice);
    return slice;
}
