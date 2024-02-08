// CVolumeViewerWithCurve.cpp
// Chao Du 2015 April
#include "CVolumeViewerWithCurve.hpp"

#include <cstddef>

#include <QSettings>
#include <opencv2/imgproc.hpp>

#include "ColorFrame.hpp"
#include "UDataManipulateUtils.hpp"

using namespace ChaoVis;

// Constructor
CVolumeViewerWithCurve::CVolumeViewerWithCurve()
    : fShowCurveBox(nullptr)
    , fHistEqBox(nullptr)
    , showCurve(true)
    , histEq(false)
    , fSplineCurveRef(nullptr)
    , fIntersectionCurveRef(nullptr)
    , fSelectedPointIndex(-1)
    , fVertexIsChanged(false)
    , fImpactRange(5)
    , fViewState(EViewState::ViewStateIdle)
{
    QSettings settings;
    colorSelector = new ColorFrame(this);
    colorSelector->setFixedSize(16, 16);
    auto color = settings.value("volumeViewer/curveColor", QColor("blue"))
                     .value<QColor>();
    colorSelector->setColor(color);
    fButtonsLayout->addWidget(colorSelector);
    connect(
        colorSelector, &ColorFrame::colorChanged, this,
        &CVolumeViewerWithCurve::UpdateView);
    connect(colorSelector, &ColorFrame::colorChanged, [](const QColor& c) {
        QSettings settings;
        settings.setValue("volumeViewer/curveColor", c);
    });

    // show curve box
    fShowCurveBox = new QCheckBox(this);
    fShowCurveBox->setChecked(true);
    connect(
        fShowCurveBox, SIGNAL(stateChanged(int)), this,
        SLOT(OnShowCurveStateChanged(int)));

    QLabel* ShowCurveLabel = new QLabel(this);
    ShowCurveLabel->setText("Show Curve");
    fButtonsLayout->addWidget(fShowCurveBox);
    fButtonsLayout->addWidget(ShowCurveLabel);

    fHistEqBox = new QCheckBox(this);
    fHistEqBox->setChecked(false);
    connect(
        fHistEqBox, SIGNAL(stateChanged(int)), this,
        SLOT(OnHistEqStateChanged(int)));

    QLabel* HistEqLabel = new QLabel(this);
    HistEqLabel->setText("HistEq");
    fButtonsLayout->addWidget(fHistEqBox);
    fButtonsLayout->addWidget(HistEqLabel);

    UpdateButtons();
}

// Set image
void CVolumeViewerWithCurve::SetImage(const QImage& nSrc)
{
    if (fImgQImage == nullptr) {
        fImgQImage = new QImage(nSrc);
    } else {
        *fImgQImage = nSrc;
    }

    fCanvas->setPixmap(QPixmap::fromImage(*fImgQImage));
    fCanvas->resize(fScaleFactor * fCanvas->pixmap(Qt::ReturnByValue).size());

    fImgMat = QImage2Mat(*fImgQImage);
    fImgMat.copyTo(fImgMatCache);

    UpdateView();
}

// Set the curve, we only hold a pointer to the original one so the data can be
// synchronized
void CVolumeViewerWithCurve::SetSplineCurve(CBSpline& nCurve)
{
    fSplineCurveRef = &nCurve;
}

// Set the intersection curve, for editing
void CVolumeViewerWithCurve::SetIntersectionCurve(CXCurve& nCurve)
{
    fIntersectionCurveRef = &nCurve;
}

// Set the impact range, for editing
void CVolumeViewerWithCurve::SetImpactRange(int nImpactRange)
{
    fImpactRange = nImpactRange;
}

// Update the B-spline curve
void CVolumeViewerWithCurve::UpdateSplineCurve(void)
{
    if (fSplineCurveRef != nullptr) {
        fSplineCurveRef->SetControlPoints(fControlPoints);
    }
}

// Update the view
void CVolumeViewerWithCurve::UpdateView(void)
{
    fImgMatCache.copyTo(fImgMat);

    if (histEq) {
        cv::cvtColor(fImgMat, fImgMat, cv::COLOR_BGR2GRAY);
        cv::equalizeHist(fImgMat, fImgMat);
        cv::cvtColor(fImgMat, fImgMat, cv::COLOR_GRAY2BGR);
    }

    if (fViewState == EViewState::ViewStateDraw) {
        // get secondary color
        int h{0}, s{0}, v{0};
        colorSelector->color().getHsv(&h, &s, &v);
        h += 180;
        if (h >= 360) {
            h = h - 360;
        }
        auto secondary = QColor::fromHsv(h, 255, 255);
        int r{0}, g{0}, b{0};
        secondary.getRgb(&r, &g, &b);
        if (fSplineCurveRef != nullptr) {
            fSplineCurveRef->DrawOnImage(fImgMat, cv::Scalar(b, g, r));
        }

        // get primary color
        colorSelector->color().getRgb(&r, &g, &b);
        for (std::size_t i = 0; i < fControlPoints.size(); ++i) {
            auto p = fControlPoints[i] - cv::Vec2f{0.5, 0.5};
            cv::circle(fImgMat, cv::Point2f(p), 1, cv::Scalar(b, g, r));
        }
    } else {
        if (fIntersectionCurveRef != nullptr && showCurve) {
            DrawIntersectionCurve();
        }
    }

    *fImgQImage = Mat2QImage(fImgMat);

    fCanvas->setPixmap(QPixmap::fromImage(*fImgQImage));
    fCanvas->resize(fScaleFactor * fCanvas->pixmap(Qt::ReturnByValue).size());

    CVolumeViewerWithCurve::UpdateButtons();

    update();  // repaint the widget
}

// Handle mouse press event
void CVolumeViewerWithCurve::mousePressEvent(QMouseEvent* e)
{
    // Instantly return if we're not editing or drawing
    if (fViewState == ViewStateIdle) {
        return;
    }

    // Get the mouse position in widget coordinates
    cv::Vec2f aWidgetLoc, aImgLoc;
    aWidgetLoc[0] = e->position().x();  // horizontal coordinate
    aWidgetLoc[1] = e->position().y();  // vertical coordinate

    // Convert to image coordinates and update the last tracked position
    WidgetLoc2ImgLoc(aWidgetLoc, aImgLoc);

    fLastPos.setX(aImgLoc[0]);
    fLastPos.setY(aImgLoc[1]);

    // Handle draw and edit
    if (fViewState == EViewState::ViewStateDraw) {
        // If left click, add control points to the curve
        if (e->buttons() & Qt::LeftButton) {  // add points

            fControlPoints.push_back(aImgLoc);
            UpdateSplineCurve();
        }
    } else if (fViewState == EViewState::ViewStateEdit) {
        // If we have points, select the one that was clicked
        if (fIntersectionCurveRef != nullptr) {
            fSelectedPointIndex =
                SelectPointOnCurve(fIntersectionCurveRef, aImgLoc);
            fIntersectionCurveRef->setLastState();
        }
    }

    UpdateView();
    e->accept();
}

// Handle mouse move event, currently only when we're editing
void CVolumeViewerWithCurve::mouseMoveEvent(QMouseEvent* event)
{
    // Instantly return if we're not editing
    if (fViewState != ViewStateEdit) {
        return;
    }

    // Get the mouse position in widget coordinates
    cv::Vec2f aWidgetLoc, aImgLoc;
    aWidgetLoc[0] = event->position().x();
    aWidgetLoc[1] = event->position().y();

    // Convert to image coordinates and get delta of change
    WidgetLoc2ImgLoc(aWidgetLoc, aImgLoc);
    Vec2<double> aDelta;
    aDelta[0] = aImgLoc[0] - fLastPos.x();
    aDelta[1] = aImgLoc[1] - fLastPos.y();

    // Update the curve if  we have change and have a selected point
    if ((aDelta[0] != 0 || aDelta[1] != 0) && (fSelectedPointIndex != -1)) {
        fIntersectionCurveRef->SetPointByDifference(
            fSelectedPointIndex, aDelta, CosineImpactFunc, fImpactRange);
        fVertexIsChanged = true;
    }

    // Redraw everything
    UpdateView();
}

// Handle mouse release event
void CVolumeViewerWithCurve::mouseReleaseEvent(QMouseEvent* /*event*/)
{
    if (fViewState != ViewStateEdit) {
        return;
    }

    if (fIntersectionCurveRef != nullptr && fVertexIsChanged) {

        // update the point positions in the path point cloud
        SendSignalPathChanged();

        fVertexIsChanged = false;
        fSelectedPointIndex = -1;
    }
}

// Handle paint event
void CVolumeViewerWithCurve::paintEvent(QPaintEvent* /*event*/) {}

// Handle setting the draw state for the curve
void CVolumeViewerWithCurve::OnShowCurveStateChanged(int state)
{
    if (state > 0)
        showCurve = true;
    else
        showCurve = false;

    UpdateView();
}

void CVolumeViewerWithCurve::OnHistEqStateChanged(int state)
{
    if (state > 0) {
        histEq = true;
    } else {
        histEq = false;
    }

    UpdateView();
}

// Convert widget location to image location
void CVolumeViewerWithCurve::WidgetLoc2ImgLoc(
    const cv::Vec2f& nWidgetLoc, cv::Vec2f& nImgLoc)
{
    float x = nWidgetLoc[0];  // horizontal coordinate
    float y = nWidgetLoc[1];  // vertical coordinate

    // the image position within its parent, the scroll area
    QPoint aP = fCanvas->pos();
    QWidget* aCurWidget = static_cast<QWidget*>(fCanvas->parent());

    // the widget loc from the event is relative to this widget, so stop here
    while (aCurWidget != this) {
        aP = aCurWidget->mapToParent(aP);
        aCurWidget = static_cast<QWidget*>(aCurWidget->parent());
    }

    x -= aP.x();
    y -= aP.y();

    // take image scale factor into account
    x /= fScaleFactor;
    y /= fScaleFactor;

    nImgLoc[0] = x;
    nImgLoc[1] = y;
}

// Select point on curve
auto CVolumeViewerWithCurve::SelectPointOnCurve(
    const CXCurve* nCurve, const cv::Vec2f& nPt) -> int
{
    const double DIST_THRESHOLD = 1.5 * fScaleFactor;

    for (std::size_t i = 0; i < nCurve->GetPointsNum(); ++i) {
        if (Norm<double>(Vec2<double>(
                nCurve->GetPoint(i)[0] - nPt[0],
                nCurve->GetPoint(i)[1] - nPt[1])) < DIST_THRESHOLD) {
            return i;
        }
    }
    return -1;  // To-Do: Change this -1 to a constant
}

// Draw intersection curve on the slice
void CVolumeViewerWithCurve::DrawIntersectionCurve(void)
{
    if (fIntersectionCurveRef != nullptr) {
        int r{0};
        int g{0};
        int b{0};
        colorSelector->color().getRgb(&r, &g, &b);
        for (std::size_t i = 0; i < fIntersectionCurveRef->GetPointsNum();
             ++i) {
            auto p0 = fIntersectionCurveRef->GetPoint(i)[0] - 0.5;
            auto p1 = fIntersectionCurveRef->GetPoint(i)[1] - 0.5;
            cv::circle(fImgMat, cv::Point2d(p0, p1), 1, cv::Scalar(b, g, r));
        }
    }
}

// Update the status of the buttons
void CVolumeViewerWithCurve::UpdateButtons(void)
{
    fZoomInBtn->setEnabled(fImgQImage != nullptr && fScaleFactor < 10.);
    fZoomOutBtn->setEnabled(fImgQImage != nullptr && fScaleFactor > 0.05);
    fResetBtn->setEnabled(
        fImgQImage != nullptr && fabs(fScaleFactor - 1.0) > 1e-6);
    fNextBtn->setEnabled(
        fImgQImage != nullptr && fViewState == EViewState::ViewStateIdle);
    fPrevBtn->setEnabled(
        fImgQImage != nullptr && fViewState == EViewState::ViewStateIdle);
    fImageIndexEdit->setEnabled(fViewState == EViewState::ViewStateIdle);
    fImageIndexEdit->SetImageIndex(fImageIndex);
}

// Disable stuff
void CVolumeViewerWithCurve::setButtonsEnabled(bool state)
{
    fZoomOutBtn->setEnabled(state);
    fZoomInBtn->setEnabled(state);
    fPrevBtn->setEnabled(state);
    fNextBtn->setEnabled(state);
    fImageIndexEdit->setEnabled(state);
    fShowCurveBox->setEnabled(state);
}
