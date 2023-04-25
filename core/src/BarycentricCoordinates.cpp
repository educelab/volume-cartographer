#include "vc/core/util/BarycentricCoordinates.hpp"

#include "vc/core/util/FloatComparison.hpp"

namespace vc = volcart;

// From Christer Ericson's Real-Time Collision Detection
// Code from:
// http://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates
cv::Vec3d vc::CartesianToBarycentric(
    const cv::Vec3d& pt,
    const cv::Vec3d& triA,
    const cv::Vec3d& triB,
    const cv::Vec3d& triC)
{
    auto v0 = triB - triA;
    auto v1 = triC - triA;
    auto v2 = pt - triA;

    auto dot00 = v0.dot(v0);
    auto dot01 = v0.dot(v1);
    auto dot11 = v1.dot(v1);
    auto dot20 = v2.dot(v0);
    auto dot21 = v2.dot(v1);
    auto invDenom = 1 / (dot00 * dot11 - dot01 * dot01);

    cv::Vec3d output;
    output[1] = (dot11 * dot20 - dot01 * dot21) * invDenom;
    output[2] = (dot00 * dot21 - dot01 * dot20) * invDenom;
    output[0] = 1.0 - output[1] - output[2];

    return output;
}

cv::Vec3d vc::BarycentricToCartesian(
    const cv::Vec3d& pt,
    const cv::Vec3d& triA,
    const cv::Vec3d& triB,
    const cv::Vec3d& triC)
{
    return pt[0] * triA + pt[1] * triB + pt[2] * triC;
}

bool vc::BarycentricPointIsInTriangle(const cv::Vec3d& pt)
{
    return (
        (pt[0] > 0.0 || AlmostEqual(pt[0], 0.0)) &&
        (pt[1] > 0.0 || AlmostEqual(pt[1], 0.0)) &&
        (pt[2] > 0.0 || AlmostEqual(pt[2], 0.0)) &&
        (pt[0] + pt[1] < 1.0 || AlmostEqual(pt[0] + pt[1], 1.0)));
}

auto vc::BarycentricNormalInterpolation(
    const cv::Vec3d& uvw,
    const cv::Vec3d& nA,
    const cv::Vec3d& nB,
    const cv::Vec3d& nC) -> cv::Vec3d
{
    return cv::normalize(
        (1 - uvw[0] - uvw[1]) * nA + uvw[1] * nB + uvw[2] * nC);
}

bool vc::CartesianPointIsInTriangle(
    const cv::Vec3d& pt,
    const cv::Vec3d& triA,
    const cv::Vec3d& triB,
    const cv::Vec3d& triC)
{
    auto bPt = CartesianToBarycentric(pt, triA, triB, triC);
    return BarycentricPointIsInTriangle(bPt);
}