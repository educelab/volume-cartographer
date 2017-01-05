// CXCurve.cpp
// Chao Du 2014 Dec
#include "CXCurve.h"

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
    for (int i = 0; i <= nImpactRange; ++i) {
        if (nIndex - i >= 0) {
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
            SetPoint(
                nIndex + i,
                fLastState[nIndex + i] +
                    nDiff *
                        ImpactFunc(
                            1.0, static_cast<double>(i) / nImpactRange, 1.0));
        }
    }
}
