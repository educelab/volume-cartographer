#include "vc/core/types/PerPixelMap.hpp"

#include <opencv2/imgcodecs.hpp>

#include "vc/core/io/PointSetIO.hpp"
#include "vc/core/io/TIFFIO.hpp"
#include "vc/core/types/Exceptions.hpp"
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
void PerPixelMap::setDimensions(size_t h, size_t w)
{
    height_ = h;
    width_ = w;
    initialize_map_();
}

void PerPixelMap::setWidth(size_t w)
{
    width_ = w;
    initialize_map_();
}

void PerPixelMap::setHeight(size_t h)
{
    height_ = h;
    initialize_map_();
}

// Get individual mappings
auto PerPixelMap::getAsPixelMap(size_t y, size_t x) -> PPM::PixelMap
{
    return {x, y, map_(y, x)};
}

// Return only valid mappings
auto PerPixelMap::getMappings() const -> std::vector<PPM::PixelMap>
{
    // Output vector
    std::vector<PixelMap> mappings;

    // For each pixel...
    for (size_t y = 0; y < height_; ++y) {
        for (size_t x = 0; x < width_; ++x) {
            // Skip this pixel if we have no mapping
            if (!hasMapping(y, x)) {
                continue;
            }

            // Put it in the vector if we go have one
            mappings.emplace_back(x, y, map_(y, x));
        }
    }

    return mappings;
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

PerPixelMap::PerPixelMap(size_t height, size_t width)
    : height_{height}, width_{width}
{
    initialize_map_();
}
auto PerPixelMap::initialized() const -> bool
{
    return width_ == map_.width() && height_ == map_.height() && width_ > 0 &&
           height_ > 0;
}
auto PerPixelMap::operator()(size_t y, size_t x) const -> const cv::Vec6d&
{
    return map_(y, x);
}
auto PerPixelMap::operator()(size_t y, size_t x) -> cv::Vec6d&
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

auto PerPixelMap::hasMapping(size_t y, size_t x) const -> bool
{
    if (mask_.empty()) {
        return true;
    }

    return mask_.at<uint8_t>(y, x) == 255;
}
auto PerPixelMap::width() const -> size_t { return width_; }
auto PerPixelMap::height() const -> size_t { return height_; }
auto PerPixelMap::mask() const -> cv::Mat { return mask_; }
void PerPixelMap::setMask(const cv::Mat& m) { mask_ = m.clone(); }
auto PerPixelMap::cellMap() const -> cv::Mat { return cellMap_; }
void PerPixelMap::setCellMap(const cv::Mat& m) { cellMap_ = m.clone(); }

auto PerPixelMap::hasVolumeID() const -> bool
{
    //    return metadata_.hasKey("volume") && !getVolumeID().empty();
}

auto PerPixelMap::getVolumeID() const -> Volume::Identifier
{
    //    return metadata_.get<Volume::Identifier>("volume");
}

void PerPixelMap::setVolumeID(const Volume::Identifier& id)
{
    //    metadata_.set<std::string>("volume", id);
    //    metadata_.save();
}

PerPixelMap::PixelMap::PixelMap(std::size_t x, std::size_t y, cv::Vec6d value)
    : x{x}
    , y{y}
    , pos{value[0], value[1], value[2]}
    , normal{value[3], value[4], value[5]}
{
}
