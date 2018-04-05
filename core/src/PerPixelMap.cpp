#include "vc/core/types/PerPixelMap.hpp"
#include "vc/core/io/PointSetIO.hpp"
#include "vc/core/types/Exceptions.hpp"

#include <opencv2/imgcodecs.hpp>

using namespace volcart;
namespace fs = boost::filesystem;

using PPM = PerPixelMap;

inline fs::path MaskPath(const fs::path& p)
{
    return p.parent_path() / (p.stem().string() + "_mask.png");
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

std::vector<PPM::PixelMap> PerPixelMap::getSortedMappings(size_t sortElement)
{

    std::vector<PixelMap> mappings;

    for (int y = 0; y < static_cast<int>(height_); ++y) {
        for (int x = 0; x < static_cast<int>(width_); ++x) {
            // Skip this pixel if we have no mapping
            if (!hasMapping(y, x)) {
                continue;
            }

            // Put it in the vector if we go have one
            mappings.emplace_back(x, y, map_(y, x));
        }
    }

    std::sort(
        mappings.begin(), mappings.end(),
        [sortElement](const PixelMap& lhs, const PixelMap& rhs) {
            return lhs.pos[sortElement] < rhs.pos[sortElement];
        });

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
}

PerPixelMap PerPixelMap::ReadPPM(const fs::path& path)
{
    PerPixelMap ppm;
    ppm.map_ = volcart::PointSetIO<cv::Vec6d>::ReadOrderedPointSet(path);
    ppm.height_ = ppm.map_.height();
    ppm.width_ = ppm.map_.width();

    ppm.mask_ = cv::imread(MaskPath(path).string(), cv::IMREAD_GRAYSCALE);
    if (ppm.mask_.empty()) {
        std::cerr << "Warning: Failed to read mask! " + MaskPath(path).string()
                  << "\n";
    }

    return ppm;
}
