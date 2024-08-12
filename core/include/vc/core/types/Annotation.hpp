#pragma once

#include <cstdint>

#include <opencv2/core.hpp>

#include "vc/core/filesystem.hpp"
#include "vc/core/types/OrderedPointSet.hpp"

namespace volcart
{

/**
 *  Annotation type
 *
 *  The first long is used to store the slice index and the second as
 *  a bit flag carrier and the two doubles contain the original point
 *  position before any manual moves.
 */
struct Annotation {
    enum Flag : std::int32_t {
        ANO_ANCHOR = 1,
        ANO_MANUAL = 2,
        ANO_USED_IN_RUN = 4
    };

    Annotation() = default;
    Annotation(std::int32_t i, Flag f, double x, double y);

    std::int32_t index;
    Flag flags;
    cv::Vec2d pt;
};

using AnnotationSet = OrderedPointSet<Annotation>;

void WriteAnnotationSet(const filesystem::path& path, const AnnotationSet& as);

auto ReadAnnotationSet(const filesystem::path& path) -> AnnotationSet;

}  // namespace volcart
