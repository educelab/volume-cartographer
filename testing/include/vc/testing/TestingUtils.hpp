#pragma once

/** @file */

#include <algorithm>

#include <opencv2/core.hpp>

namespace volcart::testing
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
    double smallTolerance = 1e-5,
    double pctDiffTolerance = 1.0);

template <
    typename Tp,
    int Cn,
    std::enable_if_t<std::is_floating_point_v<Tp>, bool> = true>
void SmallOrClose(
    const cv::Vec<Tp, Cn>& observed,
    const cv::Vec<Tp, Cn>& expected,
    Tp smallTolerance = 1e-5,
    Tp pctDiffTolerance = 1.0)
{
    for (int i = 0; i < Cn; i++) {
        SmallOrClose(
            observed[i], expected[i], smallTolerance, pctDiffTolerance);
    }
}

void ExpectNear(double observed, double expected, double pctDiffTolerance);

void AssertNear(double observed, double expected, double pctDiffTolerance);

template <class T>
bool CvMatEqual(const cv::Mat& a, const cv::Mat& b)
{
    return std::equal(
        a.template begin<T>(), a.template end<T>(), b.template begin<T>());
}

}  // namespace volcart::testing
