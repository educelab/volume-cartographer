/**
 * @file Color.hpp
 *
 * @ingroup Core
 */
#pragma once

#include <opencv2/core.hpp>

namespace volcart
{
/**
 * @brief Color type
 *
 * 8-bit unsigned int in BGR order
 */
using Color = cv::Vec3b;

/**
 * @namespace volcart::color
 * @brief Color constants
 */
namespace color
{
static const Color WHITE{255, 255, 255};
static const Color BLACK{0, 0, 0};
static const Color RED{0, 0, 255};
static const Color GREEN{0, 255, 0};
static const Color BLUE{255, 0, 0};
static const Color CYAN{255, 255, 0};
}  // namespace color
}  // namespace volcart
