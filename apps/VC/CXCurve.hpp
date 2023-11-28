// CXCurve.h
// Chao Du 2014 Dec
#pragma once

#include "MathUtils.hpp"

#include <vector>

namespace ChaoVis
{

class CXCurve
{

public:
    CXCurve(void);
    CXCurve(const CXCurve& nSrc);

    void SetSliceIndex(int nIndex) { fSliceIndex = nIndex; }
    int GetSliceIndex(void) { return fSliceIndex; }

    size_t GetPointsNum(void) const { return fPoints.size(); }

    Vec2<double> GetPoint(int nIndex) const { return fPoints[nIndex]; }
    const std::vector<Vec2<double>> GetPoints() const { return fPoints; }

    void setLastState() { fLastState = fPoints; }
    std::vector<Vec2<double>> getLastState() { return fLastState; }

    void SetPoint(int nIndex, const Vec2<double>& nPt)
    {
        fPoints[nIndex] = nPt;
    }
    void SetPointByDifference(
        int nIndex,
        const Vec2<double>& nDiff,
        double (*ImpactFunc)(double, double, double),
        int nImpactRange = 0);

    void SetPointByDifferenceMt(
        int nIndex,
        const Vec2<double>& nDiff,
        double (*ImpactFunc)(double, double, double),
        int nImpactRange = 0);

    int Get3DIndex(int nIndex) const { return f3DPointsIndex[nIndex]; }
    void Set3DIndex(int nIndex, int n3DIndex)
    {
        f3DPointsIndex[nIndex] = n3DIndex;
    }

    void InsertPoint(const Vec2<double>& nPt) { fPoints.push_back(nPt); }
    void Insert3DIndex(int n3DIndex) { f3DPointsIndex.push_back(n3DIndex); }

protected:
private:
    std::vector<Vec2<double>> fPoints;     // actual 2D points
    std::vector<Vec2<double>> fLastState;  // previous position of 2D points
    std::vector<int> f3DPointsIndex;       // mapping to 3D points index
    int fSliceIndex;

};  // class CXCurve

}  // namespace ChaoVis
