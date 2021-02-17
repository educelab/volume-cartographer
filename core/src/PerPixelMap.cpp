#include "vc/core/types/PerPixelMap.hpp"

#include <opencv2/imgcodecs.hpp>

#include "vc/core/io/PointSetIO.hpp"
#include "vc/core/io/TIFFIO.hpp"
#include "vc/core/types/Exceptions.hpp"
#include "vc/core/util/Logging.hpp"

using namespace volcart;
namespace fs = volcart::filesystem;

using PPM = PerPixelMap;

inline fs::path MaskPath(const fs::path& p)
{
    return p.parent_path() / (p.stem().string() + "_mask.png");
}

inline fs::path CellMapPath(const fs::path& p)
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
PPM::PixelMap PerPixelMap::getAsPixelMap(size_t y, size_t x)
{
    return {x, y, map_(y, x)};
}

// Return only valid mappings
std::vector<PPM::PixelMap> PerPixelMap::getMappings() const
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

PerPixelMap PerPixelMap::ReadPPM(const fs::path& path)
{
    PerPixelMap ppm;
    ppm.map_ = volcart::PointSetIO<cv::Vec6d>::ReadOrderedPointSet(path);
    ppm.height_ = ppm.map_.height();
    ppm.width_ = ppm.map_.width();

    ppm.mask_ = cv::imread(MaskPath(path).string(), cv::IMREAD_GRAYSCALE);
    if (ppm.mask_.empty()) {
        Logger()->warn("Failed to read mask: {}", MaskPath(path).string());
    }

    ppm.cellMap_ = cv::imread(CellMapPath(path).string(), cv::IMREAD_UNCHANGED);
    if (ppm.cellMap_.empty()) {
        Logger()->warn(
            "Failed to read cell map: {}", CellMapPath(path).string());
    }

    return ppm;
}
