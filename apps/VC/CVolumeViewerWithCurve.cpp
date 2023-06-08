// CVolumeViewerWithCurve.cpp
// Chao Du 2015 April
#include "CVolumeViewerWithCurve.hpp"

#include <QSettings>
#include <opencv2/imgproc.hpp>

#include "ColorFrame.hpp"
#include "UDataManipulateUtils.hpp"

#include <QCoreApplication> // To use QCoreApplication::sendEvent()

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
    , fImpactRange(8)
    , fViewState(EViewState::ViewStateIdle)
{
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(handleMouseHold()));
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
        for (size_t i = 0; i < fControlPoints.size(); ++i) {
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

void CVolumeViewerWithCurve::handleMouseHold()
{
    if (fIntersectionCurveRef != nullptr) {
        if (lastPressedButton & Qt::BackButton || lastPressedButton & Qt::ForwardButton) {
            auto p2 = GetScrollPosition() / fScaleFactor  + scrollPositionModifier;
            int closest_point =
                SelectPointOnCurve(fIntersectionCurveRef, p2, true);
            if (closest_point == -1) {
                return;
            }
            int numCurvePoints = fIntersectionCurveRef->GetPointsNum();
            double speed = 50.0;
            int pointDifference = static_cast<int>(speed / fScaleFactor);
            // std::cout << "pointDifference: " << pointDifference << " closest_point: " << closest_point << std::endl;
            if (lastPressedButton & Qt::BackButton) {
                closest_point -= pointDifference;
            }
            else if (lastPressedButton & Qt::ForwardButton) {
                closest_point += pointDifference;
            }
            closest_point = std::max(0, std::min(numCurvePoints - 1, closest_point));
            // std::cout << "closest_point: " << closest_point << std::endl;
            auto p1 = fIntersectionCurveRef->GetPoint(closest_point);
            auto v = cv::Vec2f(p1[0] - p2[0], p1[1] - p2[1]);
            auto v2 = v * 0.1;
            if (0 < std::sqrt(v2[0]*v2[0] + v2[1]*v2[1]) && std::sqrt(v2[0]*v2[0] + v2[1]*v2[1]) < (10.0 / fScaleFactor)) {
                v2 *= (10.0 / fScaleFactor) / std::sqrt(v2[0]*v2[0] + v2[1]*v2[1]);
                // check that the v2 is not overshooting p1
                if (std::abs(v2[0]) > std::abs(v[0])) {
                    v2[0] = v[0];
                }
                if (std::abs(v2[1]) > std::abs(v[1])) {
                    v2[1] = v[1];
                }
            }
            scrollPositionModifier = p2 + v2 - CleanScrollPosition((p2 + v2) * fScaleFactor) / fScaleFactor;
            // std::cout << "v2: " << v2 << " v: " << v << " p2: " << p2 << " p1: " << p1[0] << " " << p1[1] << std::endl;
            v2 += p2;
            // std::cout << "v2: " << v2 << " scrollPositionModifier: " << scrollPositionModifier << std::endl;
            ScrollToCenter(v2 * fScaleFactor);
        }
    }
}

// Handle mouse press event
void CVolumeViewerWithCurve::mousePressEvent(QMouseEvent* e)
{
    // Check if back or forward button was pressed
    if (e->buttons() & Qt::BackButton || e->buttons() & Qt::ForwardButton) {
        lastPressedButton = e->button();
        scrollPositionModifier = cv::Vec2f(0.0, 0.0);
        timer->start(25); // start timer, will trigger handleMouseHold() every 20 ms
        return;
    }
    // Instantly return if we're not editing or drawing
    if (fViewState == ViewStateIdle) {
        return;
    }

    if (lastPressedButton & Qt::NoButton) {
        return;
    }

    // Return if not left or right click
    if ( !(e->buttons() & Qt::RightButton) && !(e->buttons() & Qt::LeftButton) && !fIsMousePressed) {
        return;
    }

    // Get the mouse position in widget coordinates
    cv::Vec2f aWidgetLoc, aImgLoc;
    aWidgetLoc[0] = e->position().x();  // horizontal coordinate
    aWidgetLoc[1] = e->position().y();  // vertical coordinate

    // Convert to image coordinates
    WidgetLoc2ImgLoc(aWidgetLoc, aImgLoc);

    // Update the last tracked position
    fLastPos.setX(aImgLoc[0]);
    fLastPos.setY(aImgLoc[1]);

    // Handle draw and edit
    if (fViewState == EViewState::ViewStateDraw) {
        // If left click, add control points to the curve
        if (e->buttons() & Qt::LeftButton) {  // add points

            fControlPoints.push_back(aImgLoc);
            UpdateSplineCurve();
        }
    } else if (fViewState == EViewState::ViewStateEdit && ((e->buttons() & Qt::RightButton) || (e->buttons() & Qt::LeftButton) || fIsMousePressed)) {
        fIsMousePressed = true;
        // If we have points, select the one that was clicked
        if (fIntersectionCurveRef != nullptr) {
            bool rightClick = e->buttons() & Qt::RightButton;
            fSelectedPointIndex =
                SelectPointOnCurve(fIntersectionCurveRef, aImgLoc, rightClick);
            fIntersectionCurveRef->setLastState();

            // Set fLastPos to the position of the selected point
            if (fSelectedPointIndex >= 0) {
                fLastPos.setX(fIntersectionCurveRef->GetPoint(fSelectedPointIndex)[0]);
                fLastPos.setY(fIntersectionCurveRef->GetPoint(fSelectedPointIndex)[1]);

            // Trigger mouseMoveEvent
            QMouseEvent moveEvent(
                QEvent::MouseMove,
                e->position(),
                e->scenePosition(),
                e->globalPosition(),
                e->button(),
                e->buttons(),
                e->modifiers(),
                e->source()
            );
            QCoreApplication::sendEvent(this, &moveEvent);
            }
            return;
        }
    }

    UpdateView();
    e->accept();
}

// Handle mouse move event, currently only when we're editing
void CVolumeViewerWithCurve::mouseMoveEvent(QMouseEvent* event)
{
    // Instantly return if we're not editing
    if (fViewState != ViewStateEdit || !fIsMousePressed) {
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
void CVolumeViewerWithCurve::mouseReleaseEvent(QMouseEvent* e)
{
    if (((lastPressedButton & Qt::BackButton) && !(e->buttons() & Qt::BackButton)) || ((lastPressedButton & Qt::ForwardButton) && !(e->buttons() & Qt::ForwardButton))) {
        timer->stop();
        lastPressedButton = Qt::NoButton;  // unset the last pressed button
    }
    if (fViewState != ViewStateEdit || !fIsMousePressed) {
        return;
    }
    if (!(e->buttons() & Qt::RightButton) && !(e->button() & Qt::LeftButton)) {
        fIsMousePressed = false;
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
int CVolumeViewerWithCurve::SelectPointOnCurve(
    const CXCurve* nCurve, const cv::Vec2f& nPt, bool rightClick)
{
    const double DIST_THRESHOLD = 1.5 * fScaleFactor;

    int closestPointIndex = -1;
    double minDistance = std::numeric_limits<double>::max();

    bool shiftKeyPressed = QGuiApplication::queryKeyboardModifiers().testFlag(Qt::ShiftModifier);

    for (size_t i = 0; i < nCurve->GetPointsNum(); ++i) {
        double currentDistance = Norm<double>(Vec2<double>(
            nCurve->GetPoint(i)[0] - nPt[0],
            nCurve->GetPoint(i)[1] - nPt[1]));

        if (currentDistance < minDistance) {
            minDistance = currentDistance;
            closestPointIndex = i;
        }
    }

    if (rightClick || shiftKeyPressed || minDistance < DIST_THRESHOLD) {
        return closestPointIndex;
    } else {
        return -1; // To-Do: Change this -1 to a constant
    }
}

// Draw intersection curve on the slice
void CVolumeViewerWithCurve::DrawIntersectionCurve(void)
{
    if (fIntersectionCurveRef != nullptr) {
        int r{0};
        int g{0};
        int b{0};
        colorSelector->color().getRgb(&r, &g, &b);
        for (size_t i = 0; i < fIntersectionCurveRef->GetPointsNum(); ++i) {
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
