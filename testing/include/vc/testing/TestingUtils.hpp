#pragma once

/** @file */

#include <algorithm>

#include <opencv2/core.hpp>

namespace volcart
{
namespace testing
{

/**
 * @brief Uses Google Test to check whether double values are close enough.
 *
 * If the abs expected value is smaller than smallTolerance, check if the
 * observed value is as well. Otherwise, check whether the relative diff is
 * within pctDiffTolerance (in percentage units).
 * Taken from: http://stackoverflow.com/a/20050381/1917043
 */
void SmallOrClose(
    double observed,
    double expected,
    double smallTolerance = 0.00001,
    double pctDiffTolerance = 1.0);

void ExpectNear(double observed, double expected, double pctDiffTolerance);

void AssertNear(double observed, double expected, double pctDiffTolerance);

template <class T>
bool CvMatEqual(const cv::Mat& a, const cv::Mat& b)
{
    return std::equal(
        a.template begin<T>(), a.template end<T>(), b.template begin<T>());
}

}  // namespace testing
}  // namespace volcart
