// CBSpline.h
// Chao Du 2015 April
#pragma once

#include <vector>
#include <opencv2/opencv.hpp>
#include "CBezierCurve.h"
#include "MathUtils.h"

namespace ChaoVis
{

// REVISIT - NOTE - B-Spline can use many kinds of interpolation method (base
// function); we use Bezier curve here (Bezier-Bernstein)

class CBSpline
{

public:
    CBSpline(void);
    ~CBSpline(void);

    size_t GetNumOfControlPoints(void) const { return fControlPoints.size(); }
    Vec2<double> GetPoint(int nIndex) const { return fControlPoints[nIndex]; }

    void SetControlPoints(const std::vector<Vec2<double>>& nControlPoints);
    void SetControlPoints(const std::vector<cv::Vec2f>& nControlPoints);
    void GetSamplePoints(std::vector<Vec2<double>>& nSamplePoints);
    void GetSamplePoints(std::vector<cv::Vec2f>& nSamplePoints);

    void DrawOnImage(
        cv::Mat& nImg, const cv::Scalar& nColor = cv::Scalar(0, 0, 255));

    void Clear()
    {
        fControlPoints.clear();
        fCurveSegments.clear();
    }

protected:
private:
    void UpdateCurve(void);

private:
    std::vector<Vec2<double>> fControlPoints;
    std::vector<CBezierCurve> fCurveSegments;

};  // class CBSpline

}  // namespace ChaoVis
