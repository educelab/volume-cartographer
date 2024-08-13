#pragma once

/** @file */

#include <cstdint>

#include <opencv2/core.hpp>

#include "vc/core/filesystem.hpp"
#include "vc/core/types/OrderedPointSet.hpp"
namespace volcart
{

/** Annotation flags */
using AnnotationFlags = std::int32_t;
static constexpr AnnotationFlags ANNO_NONE = 0x0000;
static constexpr AnnotationFlags ANNO_ANCHOR = 0x0001;
static constexpr AnnotationFlags ANNO_MANUAL = 0x0002;
static constexpr AnnotationFlags ANNO_USED_IN_RUN = 0x0004;

/**
 *  @brief Segmentation annotation
 *
 *  A structure for tracking the state of segmentation points in VC GUI.
 */
struct Annotation {
    /** Default constructor */
    Annotation() = default;
    /** Constructor with members */
    Annotation(std::int32_t i, AnnotationFlags f, double x, double y);

    /** @brief Slice index */
    std::int32_t index{};
    /** @see AnnotationFlags */
    AnnotationFlags flags{};
    /** @brief The original point position before manual moves */
    cv::Vec2d pt;
};

/** @brief Ordered annotation collection */
using AnnotationSet = OrderedPointSet<Annotation>;

/** @brief Write an AnnotationSet to disk */
void WriteAnnotationSet(const filesystem::path& path, const AnnotationSet& as);

/** @brief Load an AnnotationSet from disk */
auto ReadAnnotationSet(const filesystem::path& path) -> AnnotationSet;

}  // namespace volcart
