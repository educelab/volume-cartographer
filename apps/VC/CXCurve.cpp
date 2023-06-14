// CXCurve.cpp
// Chao Du 2014 Dec
#include "CXCurve.hpp"
#include <omp.h>

using namespace ChaoVis;

// Constructor
CXCurve::CXCurve(void) {}

// Copy constructor
CXCurve::CXCurve(const CXCurve& nSrc)
{
    fSliceIndex = nSrc.fSliceIndex;
    fPoints = nSrc.fPoints;
    fLastState = fPoints;
    f3DPointsIndex = nSrc.f3DPointsIndex;
}

// Change point position, use different weighting function
void CXCurve::SetPointByDifference(
    int nIndex,
    const Vec2<double>& nDiff,
    double (*ImpactFunc)(double, double, double),
    int nImpactRange)
{
    // Multi-threading is faster only when the number of points is large enough
    if (nImpactRange > 1) {
        SetPointByDifferenceMt(nIndex, nDiff, ImpactFunc, nImpactRange);
        return;
    }
    auto it = fPoints.begin();
    for (int i = 0; i <= nImpactRange; ++i) {
        if (nIndex - i >= 0) {
            auto pt = it;
            std::advance(pt, nIndex - i);
            SetPoint(
                nIndex - i,
                fLastState[nIndex - i] +
                    nDiff *
                        ImpactFunc(
                            1.0, static_cast<double>(i) / nImpactRange, 1.0));
        }
        if (i == 0) {
            continue;
        }
        if (nIndex + i < static_cast<int>(fPoints.size())) {
            auto pt = it;
            std::advance(pt, nIndex + i);
            SetPoint(
                nIndex + i,
                fLastState[nIndex + i] +
                    nDiff *
                        ImpactFunc(
                            1.0, static_cast<double>(i) / nImpactRange, 1.0));
        }
    }
}

void CXCurve::SetPointByDifferenceMt(
    int nIndex,
    const Vec2<double>& nDiff,
    double (*ImpactFunc)(double, double, double),
    int nImpactRange)
{
    #pragma omp parallel for
    for (int i = -nImpactRange; i <= nImpactRange; ++i) {
        int current_index = nIndex + i;
        if (current_index >= 0 && current_index < static_cast<int>(fPoints.size())) {
            SetPoint(
                current_index,
                fLastState[current_index] +
                    nDiff *
                        ImpactFunc(
                            1.0, static_cast<double>(std::abs(i)) / nImpactRange, 1.0));
        }
    }
}
