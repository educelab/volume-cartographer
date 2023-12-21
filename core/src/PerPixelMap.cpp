#include "vc/core/types/PerPixelMap.hpp"

#include <opencv2/imgcodecs.hpp>

#include "vc/core/io/PointSetIO.hpp"
#include "vc/core/io/TIFFIO.hpp"
#include "vc/core/types/Exceptions.hpp"
#include "vc/core/util/Iteration.hpp"
#include "vc/core/util/Logging.hpp"

using namespace volcart;
namespace fs = volcart::filesystem;

using PPM = PerPixelMap;

inline auto MaskPath(const fs::path& p) -> fs::path
{
    return p.parent_path() / (p.stem().string() + "_mask.png");
}

inline auto CellMapPath(const fs::path& p) -> fs::path
{
    return p.parent_path() / (p.stem().string() + "_cellmap.tif");
}

///// Metadata /////
void PerPixelMap::setDimensions(std::size_t h, std::size_t w)
{
    height_ = h;
    width_ = w;
    initialize_map_();
}

void PerPixelMap::setWidth(std::size_t w)
{
    width_ = w;
    initialize_map_();
}

void PerPixelMap::setHeight(std::size_t h)
{
    height_ = h;
    initialize_map_();
}

// Get individual mappings
auto PerPixelMap::getAsPixelMap(std::size_t y, std::size_t x) -> PPM::PixelMap
{
    return {y, x, map_(y, x)};
}

// Return only valid mappings
auto PerPixelMap::getMappings() const -> std::vector<PixelMap>
{
    // Output vector
    std::vector<PixelMap> mappings;
    mappings.reserve(numMappings());

    // For each pixel...
    for (auto [y, x] : range2D(height_, width_)) {
        // Skip this pixel if we have no mapping
        if (!hasMapping(y, x)) {
            continue;
        }

        // Put it in the vector if we go have one
        mappings.emplace_back(y, x, map_(y, x));
    }

    return mappings;
}

auto PerPixelMap::getMappingCoords() const -> std::vector<Coord2D>
{
    // Get the list of valid coordinates
    std::vector<Coord2D> idxs;
    idxs.reserve(numMappings());
    for (auto [y, x] : range2D(height_, width_)) {
        if (hasMapping(y, x)) {
            idxs.emplace_back(y, x);
        }
    }

    return idxs;
}

auto PerPixelMap::numMappings() const -> std::size_t
{
    if (mask_.empty()) {
        return width_ * height_;
    }

    return cv::countNonZero(mask_);
}

// Initialize map
void PerPixelMap::initialize_map_()
{
    if (height_ > 0 && width_ > 0) {
        map_ = volcart::OrderedPointSet<cv::Vec6d>::Fill(
            width_, height_, {0, 0, 0, 0, 0, 0});
    }
}

///// Disk IO /////
void PerPixelMap::WritePPM(const fs::path& path, const PerPixelMap& map)
{
    volcart::PointSetIO<cv::Vec6d>::WriteOrderedPointSet(path, map.map_);

    if (!map.mask_.empty()) {
        cv::imwrite(MaskPath(path).string(), map.mask_);
    }

    if (!map.cellMap_.empty()) {
        tiffio::WriteTIFF(CellMapPath(path), map.cellMap_);
    }
}

auto PerPixelMap::ReadPPM(const fs::path& path) -> PerPixelMap
{
    PerPixelMap ppm;
    ppm.map_ = volcart::PointSetIO<cv::Vec6d>::ReadOrderedPointSet(path);
    ppm.height_ = ppm.map_.height();
    ppm.width_ = ppm.map_.width();

    auto maskPath = MaskPath(path);
    if (fs::exists(maskPath)) {
        ppm.mask_ = cv::imread(maskPath.string(), cv::IMREAD_GRAYSCALE);
    }
    if (ppm.mask_.empty()) {
        Logger()->warn("Failed to read mask: {}", MaskPath(path).string());
    }

    auto cellMapPath = CellMapPath(path);
    if (fs::exists(cellMapPath)) {
        ppm.cellMap_ = tiffio::ReadTIFF(cellMapPath);
    }
    if (ppm.cellMap_.empty()) {
        Logger()->warn(
            "Failed to read cell map: {}", CellMapPath(path).string());
    }

    return ppm;
}

PerPixelMap::PerPixelMap(std::size_t height, std::size_t width)
    : height_{height}, width_{width}
{
    initialize_map_();
}
auto PerPixelMap::initialized() const -> bool
{
    return width_ == map_.width() && height_ == map_.height() && width_ > 0 &&
           height_ > 0;
}
auto PerPixelMap::operator()(std::size_t y, std::size_t x) const
    -> const cv::Vec6d&
{
    return map_(y, x);
}
auto PerPixelMap::operator()(std::size_t y, std::size_t x) -> cv::Vec6d&
{
    return map_(y, x);
}

auto PerPixelMap::getMapping(std::size_t y, std::size_t x) const
    -> const cv::Vec6d&
{
    return map_(y, x);
}

auto PerPixelMap::getMapping(std::size_t y, std::size_t x) -> cv::Vec6d&
{
    return map_(y, x);
}

auto PerPixelMap::hasMapping(std::size_t y, std::size_t x) const -> bool
{
    if (mask_.empty()) {
        return true;
    }

    return mask_.at<uint8_t>(y, x) == 255;
}
auto PerPixelMap::width() const -> std::size_t { return width_; }
auto PerPixelMap::height() const -> std::size_t { return height_; }
auto PerPixelMap::mask() const -> cv::Mat { return mask_; }
void PerPixelMap::setMask(const cv::Mat& m) { mask_ = m.clone(); }
auto PerPixelMap::cellMap() const -> cv::Mat { return cellMap_; }
void PerPixelMap::setCellMap(const cv::Mat& m) { cellMap_ = m.clone(); }

auto PerPixelMap::Crop(
    const PerPixelMap& map,
    std::size_t originY,
    std::size_t originX,
    std::size_t height,
    std::size_t width) -> PerPixelMap
{
    // Check that origin is in bounds
    if (originY >= map.height_ or originX >= map.width_) {
        throw std::runtime_error("Crop origin out-of-bounds");
    }

    // Limit the output dimensions
    auto maxX = std::min(map.width_, originX + width);
    auto maxY = std::min(map.height_, originY + height);
    height = maxY - originY;
    width = maxX - originX;

    // Create output
    PerPixelMap out(height, width);

    // Copy mappings
    for (auto [y, x] : range2D(originY, maxY, originX, maxX)) {
        auto yy = y - originY;
        auto xx = x - originX;
        out(yy, xx) = map(y, x);
    }

    // Copy mask
    const cv::Rect roi(originX, originY, width, height);
    if (not map.mask_.empty()) {
        map.mask_(roi).copyTo(out.mask_);
    }

    // Copy cell map
    if (not map.cellMap_.empty()) {
        map.cellMap_(roi).copyTo(out.cellMap_);
    }

    return out;
}
