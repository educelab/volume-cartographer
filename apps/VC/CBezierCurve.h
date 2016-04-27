// CBezierCurve.h
// Chao Du 2015 April
#ifndef _CBEZIERCURVE_H_
#define _CBEZIERCURVE_H_

#include <vector>

#include "mathUtils.h"

#include <opencv2/opencv.hpp>

namespace ChaoVis
{

// REVISIT - NOTE - we use order 3 Bezier curve, which has 4 control points and
// 3 convex line segments

class CBezierCurve
{

public:
    CBezierCurve(int nSampleInterval = 5.0);
    CBezierCurve(const std::vector<Vec2<double>> &nControlPoints,
                 int nSampleInterval = 5.0);
    ~CBezierCurve(void);

    void SetControlPoints(const std::vector<Vec2<double>> &nControlPoints);
    void SetControlPoints(const std::vector<cv::Vec2f> &nControlPoints);
    void GetSamplePoints(std::vector<Vec2<double>> &nSamplePoints);
    void GetSamplePoints(std::vector<cv::Vec2f> &nSamplePoints);

    void DrawOnImage(cv::Mat &nImg,
                     const cv::Scalar &nColor = cv::Scalar(0, 0, 255));

protected:
private:
    int fNumControlPoints;  // constantly equals 4
    std::vector<Vec2<double>> fControlPoints;
    double fSampleInterval;

};  // class CBezierCurve

}  // namespace ChaoVis

#endif  // _CBEZIERCURVE_H_
