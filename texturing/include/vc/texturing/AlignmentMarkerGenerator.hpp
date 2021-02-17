#pragma once

/** @file */

#include <vector>

#include "vc/core/types/TexturedMesh.hpp"

namespace volcart::texturing
{
/**
 * @brief Add 2D alignment markers to textured meshes
 *
 * Takes as input a list of textured meshes and a list of alignment line
 * segments. The line segments are assumed to pass through one or more of the
 * textured meshes. The intersection points between all segments and meshes are
 * computed, and a colored circle is drawn on the texture images at the point of
 * each intersection. Returns a new set of texture images, modified with the
 * colored alignment markers.
 *
 * @note Output images will be 8bpc and 3-channel (BGR)
 *
 * @warning Line segments do not have to be parallel to each other, nor do they
 * have to be perpendicular to the surfaces of the meshes. Improperly setting
 * the position of segment end points can result in alignment markers that make
 * no sense.
 */
class AlignmentMarkerGenerator
{
public:
    /** @brief Defines a marker intersection line segment */
    struct LineSegment {
        /** Default constructor */
        LineSegment() = default;
        /** Constructor with member components */
        LineSegment(const cv::Vec3d& a, const cv::Vec3d& b, cv::Scalar c = 0);

        /** Start point */
        cv::Vec3d a;
        /** End point */
        cv::Vec3d b;
        /** Marker color */
        cv::Scalar color;
    };

    /** @brief Set the input textured meshes */
    void setInputMeshes(std::vector<volcart::TexturedMesh> m);

    /** @brief Set the marker intersection line segments */
    void setLineSegments(std::vector<LineSegment> r);

    /**
     * @brief Set the radius, in pixels, of the drawn markers.
     *
     * Default value: 5
     */
    void setMarkerRadius(int r);

    /**
     * @brief If `true`, randomly assign a color to each marker.
     *
     * Default: `true`
     */
    void setMarkerUseRandomColor(bool b);

    /** @brief Compute the marked images */
    std::vector<cv::Mat> compute();

    /** @brief Get the computed marked images */
    std::vector<cv::Mat> getMarkedImages() const;

private:
    /** Input meshes */
    std::vector<volcart::TexturedMesh> input_;
    /** Intersection line segments */
    std::vector<LineSegment> lineSegments_;
    /** Marker radius */
    int markerRadius_{5};
    /** Generate random colors */
    bool markerRandomColor_{true};
    /** Output images */
    std::vector<cv::Mat> output_;
};

}  // namespace volcart::texturing