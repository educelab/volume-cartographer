#pragma once

/** @file */

#include <opencv2/core.hpp>

#include "vc/core/types/Color.hpp"

namespace volcart
{
namespace texturing
{
/**
 * @author Kyra Seevers
 *
 * @brief Add a scale marker (ruler or reference image) to a Texture image
 *
 * Adds a ruler bar or a reference image to a Texture image to aid with
 * identifying features and text. To be accurate, the pixel size (in microns)
 * must be set for all provided images.
 *
 * Input images must be 1 or 3 channels. Output images will be 8bpc RGB.
 */
class ScaleMarkerGenerator
{
public:
    /** @brief Scale marker type */
    enum class Type { Metric = 0, Imperial, ReferenceImage };

    /** Default constructor */
    ScaleMarkerGenerator() = default;

    /** @brief Set the scale marker type */
    void setScaleType(Type t) { type_ = t; }

    /** @brief Set the input texture image */
    void setInputImage(const cv::Mat& i) { inputImg_ = i; }

    /** @brief Set the input texture image pixel size (in microns) */
    void setInputImagePixelSize(double s) { inputImgPixSize_ = s; }

    /**
     * @brief Set the scale bar (ruler) color
     *
     * Used when the scale type is Type::Imperial or Type::Metric.
     */
    void setScaleBarColor(const Color& c) { scaleBarColor_ = c; }

    /**
     * @brief Set the scale reference image
     *
     * Used when the scale type is Type::ReferenceImage.
     */
    void setReferenceImage(const cv::Mat& i) { refImg_ = i; }

    /** @brief Set the reference image pixel size (in microns) */
    void setReferenceImagePixelSize(double s) { refImgPixSize_ = s; }

    /**
     * @brief Compute scale marker and combine with input texture image
     *
     * @throws std::domain_error
     */
    cv::Mat compute();

    /**
     * @brief Get the computed output image
     *
     * Returned image is the input texture image with the scale marker.
     */
    cv::Mat getOutputImg() { return outputImg_; }

    /**
     * @brief Get the computed scale marker image
     *
     * Returned image is only the scale marker.
     */
    cv::Mat getOutputScaleImage() { return scaleImg_; }

private:
    /** Scale marker type */
    Type type_{Type::Metric};

    /** Input texture image */
    cv::Mat inputImg_;
    /** Pixel size of input texture image (in microns) */
    double inputImgPixSize_{1.0};

    /** Generated scale image */
    cv::Mat scaleImg_;
    /** Output texture image + scale image */
    cv::Mat outputImg_;

    /** Color of scale bar */
    Color scaleBarColor_{volcart::color::WHITE};

    /** Input reference image */
    cv::Mat refImg_;
    /** Pixel size of input reference image (in microns) */
    double refImgPixSize_{1.0};

    /** Conversion scale factor between constituent images */
    double scaleFactor_{1.0};
    /** Text version of scale unit */
    std::string scaleUnit_{"cm"};

    /** Resize the reference image to resolution of the input texture image */
    cv::Mat resize_ref_image_();

    /** Generate a scale bar at the resolution of the input texture image */
    cv::Mat generate_scale_bar_();

    /** Draw a unit label "value + scaleUnit_" at the given position */
    void draw_tick_label_(
        cv::Mat img, const std::string& value, const cv::Point& position);
};
}  // namespace texturing
}  // namespace volcart
