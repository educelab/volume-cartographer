#include "vc/core/types/UVMap.hpp"

/** Top-left UV Origin */
const static cv::Vec2d ORIGIN_TOP_LEFT(0, 0);
/** Top-right UV Origin */
const static cv::Vec2d ORIGIN_TOP_RIGHT(1, 0);
/** Bottom-left UV Origin */
const static cv::Vec2d ORIGIN_BOTTOM_LEFT(0, 1);
/** Bottom-right UV Origin */
const static cv::Vec2d ORIGIN_BOTTOM_RIGHT(1, 1);

using namespace volcart;

void UVMap::set(size_t id, const cv::Vec2d& uv, const Origin& o)
{
    // transform to be relative to top-left
    cv::Vec2d transformed;
    cv::absdiff(uv, origin_vector_(o), transformed);
    map_[id] = transformed;
}

void UVMap::set(size_t id, const cv::Vec2d& uv) { set(id, uv, origin_); }

cv::Vec2d UVMap::get(size_t id, const Origin& o)
{
    auto it = map_.find(id);
    if (it != map_.end()) {
        // transform to be relative to the provided origin
        cv::Vec2d transformed;
        cv::absdiff(it->second, origin_vector_(o), transformed);
        return transformed;
    } else {
        return NULL_MAPPING;
    }
}

cv::Vec2d UVMap::get(size_t id) { return get(id, origin_); }

cv::Vec2d UVMap::origin_vector_(const Origin& o)
{
    switch (o) {
        case Origin::TopLeft:
            return ORIGIN_TOP_LEFT;
        case Origin::TopRight:
            return ORIGIN_TOP_RIGHT;
        case Origin::BottomLeft:
            return ORIGIN_BOTTOM_LEFT;
        case Origin::BottomRight:
            return ORIGIN_BOTTOM_RIGHT;
    }
}