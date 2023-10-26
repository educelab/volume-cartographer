// CVolumeViewerWithCurve.h
// Chao Du 2015 April
#pragma once

#include <QCheckBox>
#include <vector>
#include <opencv2/core.hpp>
#include "CBSpline.hpp"
#include "CVolumeViewer.hpp"
#include "CXCurve.hpp"
#include "ColorFrame.hpp"
#include "SegmentationStruct.hpp"

#include <QList>
#include <QGraphicsEllipseItem>
#include <QGraphicsView>
#include <QGraphicsScene>

namespace ChaoVis
{

// REVISIT - NOTE - since there are two modes, edit and draw, for the
// application, we need to add corresponding modes to the widget, too.

class CVolumeViewerWithCurve : public CVolumeViewer
{

    Q_OBJECT

public:
    enum EViewState {
        ViewStateEdit,  // edit mode
        ViewStateDraw,  // draw mode
        ViewStateIdle   // idle mode
    };  

public:
    CVolumeViewerWithCurve(std::unordered_map<std::string, SegmentationStruct>& nSegStructMapRef);
    ~CVolumeViewerWithCurve();

    virtual void SetImage(const QImage& nSrc);

    // for drawing mode
    void SetSplineCurve(CBSpline& nCurve);
    void UpdateSplineCurve(void);
    void ResetSplineCurve(void) { fControlPoints.clear(); }
    // for editing mode
    void SetIntersectionCurve(CXCurve& nCurve);
    void SetImpactRange(int nImpactRange);
    void SetScanRange(int nScanRange);
    void SetSliceIndexToolStart(int index) { sliceIndexToolStart = index; }

    void UpdateView();
    void SetShowCurve(bool b) { showCurve = b; }
    void toggleShowCurveBox();

    void SetViewState(EViewState nViewState) { fViewState = nViewState; }
    EViewState GetViewState(void) { return fViewState; }

    void setButtonsEnabled(bool state);

protected:
    bool eventFilter(QObject* watched, QEvent* event);
    void mousePressEvent(QMouseEvent* event);
    void mouseMoveEvent(QMouseEvent* event);
    void mouseReleaseEvent(QMouseEvent* event);
    void paintEvent(QPaintEvent* event);
    void UpdateButtons(void);

private slots:
    void OnShowCurveStateChanged(int state);
    void OnHistEqStateChanged(int state);
    void handleMouseHold();

private:
    void WidgetLoc2ImgLoc(const cv::Vec2f& nWidgetLoc, cv::Vec2f& nImgLoc);

    std::pair<int, std::string> SelectPointOnCurves(const cv::Vec2f& nPt, bool rightClick, bool selectGlobally=false);

    void DrawIntersectionCurve(QGraphicsScene* scene);
    void DrawControlPoints(QGraphicsScene* scene);

private slots:

signals:
    void SendSignalPathChanged(void);

private:
    // for interaction
    QTimer *timer;
    Qt::MouseButton lastPressedButton{Qt::NoButton};
    cv::Vec2f scrollPositionModifier{cv::Vec2f(0.0, 0.0)};

    // for drawing
    ColorFrame* colorSelector{nullptr};
    ColorFrame* colorSelectorCompute{nullptr};
    ColorFrame* colorSelectorHighlight{nullptr};
    QCheckBox* fShowCurveBox;
    QCheckBox* fHistEqBox;
    bool showCurve;
    bool histEq;
    int fwdBackMsJump;
    std::unordered_map<std::string, SegmentationStruct>& fSegStructMapRef;
    CBSpline* fSplineCurveRef;
    std::vector<cv::Vec2f> fControlPoints;

    // for editing
    CXCurve* fIntersectionCurveRef;
    int fSelectedPointIndex;
    std::string fSelectedSegID;
    int sliceIndexToolStart;

    bool fVertexIsChanged;
    bool fIsMousePressed{false};
    QPointF fLastPos;  // last mouse position on the image
    int fImpactRange;  // how many points a control point movement can affect

    // pan handling
    bool wantsPanning{false};
    bool isPanning{false};
    bool rightPressed{false};
    int panStartX, panStartY;

    // image drawn
    cv::Mat fImgMat;
    cv::Mat fImgMatCache;

    EViewState fViewState;

    // Global or class-level storage for ellipse items
    QList<QGraphicsEllipseItem*> ellipseItems;
    QList<QGraphicsEllipseItem*> controlPointItems;
};  // class CVolumeViewerWithCurve

}  // namespace ChaoVis
