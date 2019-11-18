#pragma once

#include <opencv2/core.hpp>

namespace volcart
{

/**
 * @brief Convert a Cartesian coordinate to barycentric coordinate relative to
 * the given triangle
 *
 * @param pt Cartesian coordinate
 * @param triA Position of the first vertex in the triangle
 * @param triB Position of the second vertex in the triangle
 * @param triC Position of the third vertex in the triangle
 */
cv::Vec3d CartesianToBarycentric(
    const cv::Vec3d& pt,
    const cv::Vec3d& triA,
    const cv::Vec3d& triB,
    const cv::Vec3d& triC);

/**
 * @brief Convert a barycentric coordinate relative to the given triangle to a
 * Cartesian coordinate
 *
 * @param pt Barycentric coordinate
 * @param triA Position of the first vertex in the triangle
 * @param triB Position of the second vertex in the triangle
 * @param triC Position of the third vertex in the triangle
 */
cv::Vec3d BarycentricToCartesian(
    const cv::Vec3d& pt,
    const cv::Vec3d& triA,
    const cv::Vec3d& triB,
    const cv::Vec3d& triC);

/**
 * @brief Check whether a barycentric coordinate is bounded by the given
 * triangle
 */
bool BarycentricPointIsInTriangle(const cv::Vec3d& pt);

/**
 * @brief Check whether a Cartesian coordinate is bounded by the given triangle
 */
bool CartesianPointIsInTriangle(
    const cv::Vec3d& pt,
    const cv::Vec3d& triA,
    const cv::Vec3d& triB,
    const cv::Vec3d& triC);

}  // namespace volcart
