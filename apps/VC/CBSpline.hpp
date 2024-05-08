// CBSpline.h
// Chao Du 2015 April
#pragma once

#include <cstddef>
#include <vector>

#include <opencv2/core.hpp>

#include "CBezierCurve.hpp"
#include "MathUtils.hpp"
#include <QGraphicsEllipseItem>
#include <QGraphicsView>
#include <QGraphicsScene>

namespace ChaoVis
{

// REVISIT - NOTE - B-Spline can use many kinds of interpolation method (base
// function); we use Bezier curve here (Bezier-Bernstein)

class CBSpline
{

public:
    CBSpline(void);
    ~CBSpline(void);

    std::size_t GetNumOfControlPoints(void) const
    {
        return fControlPoints.size();
    }
    Vec2<double> GetPoint(int nIndex) const { return fControlPoints[nIndex]; }

    void SetControlPoints(const std::vector<Vec2<double>>& nControlPoints);
    void SetControlPoints(const std::vector<cv::Vec2f>& nControlPoints);
    void GetSamplePoints(std::vector<Vec2<double>>& nSamplePoints);
    void GetSamplePoints(std::vector<cv::Vec2f>& nSamplePoints);

    void DrawOnImage(QGraphicsScene* scene, const QColor& color = QColor(255, 0, 0));


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
