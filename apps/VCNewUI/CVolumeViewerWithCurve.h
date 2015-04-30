// CVolumeViewerWithCurve.h
// Chao Du 2015 April
#ifndef _CVOLUMEVIEWERWITHCURVE_H_
#define _CVOLUMEVIEWERWITHCURVE_H_

#include "CVolumeViewer.h"
#include "CBSpline.h"
#include <vector>
#include <opencv2/opencv.hpp>

namespace ChaoVis {

class CVolumeViewerWithCurve : public CVolumeViewer {

public:
    CVolumeViewerWithCurve( void );
    ~CVolumeViewerWithCurve( void );

    virtual void SetImage( const QImage &nSrc );

    void SetCurve( CBSpline &nCurve );
    void UpdateCurve( void );
    void UpdateView( void );

protected:
    void mousePressEvent( QMouseEvent *event );
    void mouseMoveEvent( QMouseEvent *event );
    void paintEvent( QPaintEvent *event );

private:
    void WidgetLoc2ImgLoc( const cv::Vec2f &nWidgetLoc,
                           cv::Vec2f       &nImgLoc );

private slots:

signals:

private:
    CBSpline    *fCurveRef;
    std::vector< cv::Vec2f >
                fControlPoints;
    cv::Mat     fImgMat;
    cv::Mat     fImgMatCache;

}; // class CVolumeViewerWithCurve

} // namespace ChaoVis

#endif // _CVOLUMEVIEWERWITHCURVE_H_
