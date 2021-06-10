#pragma once

/** @file */

#include <opencv2/core.hpp>

namespace volcart
{

/**
 * @brief Built-in color maps
 *
 * @ingroup Util
 */
enum class ColorMap {
    /** \image html magma.png */
    Magma = 0,
    /** \image html inferno.png */
    Inferno,
    /** \image html plasma.png */
    Plasma,
    /** \image html viridis.png */
    Viridis,
    /** \image html phase.png */
    Phase,
    /** \image html bwr.png */
    BWR
};

/** @brief Get the string name of a ColorMap */
std::string ColorMapToString(ColorMap cm);

/** @brief Get a ColorMap from its string name */
ColorMap ColorMapFromString(const std::string& str);

/**
 * @brief Get a color map LUT for use with ApplyLUT
 *
 * @ingroup Util
 */
cv::Mat GetColorMapLUT(ColorMap cm, std::size_t bins = 256);

/**
 * @brief Get a color map LUT for use with ApplyLUT
 *
 * @ingroup Util
 */
cv::Mat GetColorMapLUT(const std::string& name, std::size_t bins = 256);

}  // namespace volcart