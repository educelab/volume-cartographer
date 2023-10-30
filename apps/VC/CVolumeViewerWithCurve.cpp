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
CVolumeViewerWithCurve::CVolumeViewerWithCurve(std::unordered_map<std::string, SegmentationStruct>& nSegStructMapRef)
    : fShowCurveBox(nullptr)
    , fHistEqBox(nullptr)
    , showCurve(true)
    , histEq(false)
    , fSplineCurveRef(nullptr)
    , fIntersectionCurveRef(nullptr)
    , fSelectedPointIndex(-1)
    , fVertexIsChanged(false)
    , fImpactRange(8)
    , fSegStructMapRef(nSegStructMapRef)
{
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(handleMouseHold()));
    QSettings settings;
    colorSelector = new ColorFrame(this);
    colorSelector->setFixedSize(16, 16);
    auto color = settings.value("volumeViewer/curveColor", QColor("green"))
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
    colorSelectorCompute = new ColorFrame(this);
    colorSelectorCompute->setFixedSize(16, 16);
    auto colorCompute = settings.value("volumeViewer/computeColor", QColor("blue"))
                     .value<QColor>();
    colorSelectorCompute->setColor(colorCompute);
    fButtonsLayout->addWidget(colorSelectorCompute);
    connect(
        colorSelectorCompute, &ColorFrame::colorChanged, this,
        &CVolumeViewerWithCurve::UpdateView);
    connect(colorSelectorCompute, &ColorFrame::colorChanged, [](const QColor& c) {
        QSettings settings;
        settings.setValue("volumeViewer/computeColor", c);
    });
    colorSelectorHighlight = new ColorFrame(this);
    colorSelectorHighlight->setFixedSize(16, 16);
    auto colorHighlight = settings.value("volumeViewer/computeHighlight", QColor("red"))
                     .value<QColor>();
    colorSelectorHighlight->setColor(colorHighlight);
    fButtonsLayout->addWidget(colorSelectorHighlight);
    connect(
        colorSelectorHighlight, &ColorFrame::colorChanged, this,
        &CVolumeViewerWithCurve::UpdateView);
    connect(colorSelectorHighlight, &ColorFrame::colorChanged, [](const QColor& c) {
        QSettings settings;
        settings.setValue("volumeViewer/computeHighlight", c);
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

    QSettings settingsJump("VC.ini", QSettings::IniFormat);
    fwdBackMsJump = settingsJump.value("viewer/fwd_back_step_ms", 25).toInt();

    UpdateButtons();

    this->installEventFilter(this);
}

// Destructor
CVolumeViewerWithCurve::~CVolumeViewerWithCurve()
{
    if (fImgQImage != nullptr) {
        delete fImgQImage;
    }
    timer->stop();
    delete timer;
}


void CVolumeViewerWithCurve::SetImage(const QImage& nSrc)
{
    if (fImgQImage == nullptr) {
        fImgQImage = new QImage(nSrc);
    } else {
        *fImgQImage = nSrc;
    }

    // Create a QPixmap from the QImage
    QPixmap pixmap = QPixmap::fromImage(*fImgQImage);

    // Add the QPixmap to the scene as a QGraphicsPixmapItem
    if(fBaseImageItem) {
        // If the item already exists, remove it from the scene
        fScene->removeItem(fBaseImageItem);
        delete fBaseImageItem; // Delete the old item
    }
    fBaseImageItem = fScene->addPixmap(pixmap);

    UpdateButtons();
    update();
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
    fGraphicsView->showCurrentImpactRange(nImpactRange);
}

// Set the scan range
void CVolumeViewerWithCurve::SetScanRange(int nScanRange)
{
    fScanRange = nScanRange;
    fGraphicsView->showCurrentScanRange(nScanRange);
}

// Return to the slice that the tool was started on 
void CVolumeViewerWithCurve::ReturnToSliceIndexToolStart()
{
    SendSignalOnLoadAnyImage(sliceIndexToolStart);
}

// Update the B-spline curve
void CVolumeViewerWithCurve::UpdateSplineCurve(void)
{
    if (fSplineCurveRef != nullptr) {
        fSplineCurveRef->SetControlPoints(fControlPoints);
    }
}

void CVolumeViewerWithCurve::UpdateView()
{
    // Remove all existing ellipses and lines
    QList<QGraphicsItem*> allItems = fScene->items();
    for(QGraphicsItem *item : allItems)
    {
        if(dynamic_cast<QGraphicsEllipseItem*>(item) || dynamic_cast<QGraphicsLineItem*>(item))
        {
            fScene->removeItem(item);
            delete item;
        }
    }

    if (fViewState == EViewState::ViewStateDraw) {
        // Get secondary color
        int h{0}, s{0}, v{0};
        colorSelector->color().getHsv(&h, &s, &v);
        h += 180;
        if (h >= 360) {
            h = h - 360;
        }
        auto secondary = QColor::fromHsv(h, 255, 255);
        
        if (fSplineCurveRef != nullptr) {
            // Assuming DrawOnImage now works on QImage or QGraphicsScene
           fSplineCurveRef->DrawOnImage(fScene, secondary);
        }
        DrawControlPoints(fScene);
    }
    
    if (showCurve) {
        // qDebug() << "showCurve";
        DrawIntersectionCurve(fScene);
    }

    // If we have an image, draw it
    if (fImgQImage != nullptr) {
        CVolumeViewerWithCurve::UpdateButtons();
    }

    update();  // Repaint the widget
}

void CVolumeViewerWithCurve::handleMouseHold()
{
    if (lastPressedButton & Qt::BackButton || lastPressedButton & Qt::ForwardButton) {
        auto p2 = GetScrollPosition() / fScaleFactor  + scrollPositionModifier;

        auto res = SelectPointOnCurves(p2, false, true);
        fSelectedPointIndex = res.first;
        fSelectedSegID = res.second;
        
        if (fSelectedPointIndex == -1) {
            return;
        }
        // fIntersectionCurveRef from the fSelectedSegID
        int numCurvePoints = fSegStructMapRef[fSelectedSegID].fIntersectionCurve.GetPointsNum();
        double speed = 50.0;
        int pointDifference = static_cast<int>(speed / fScaleFactor);
        // qDebug() << "pointDifference: " << pointDifference << " fSelectedPointIndex: " << fSelectedPointIndex;
        if (lastPressedButton & Qt::BackButton) {
            fSelectedPointIndex -= pointDifference;
        }
        else if (lastPressedButton & Qt::ForwardButton) {
            fSelectedPointIndex += pointDifference;
        }
        fSelectedPointIndex = std::max(0, std::min(numCurvePoints - 1, fSelectedPointIndex));
        // qDebug() << "fSelectedPointIndex: " << fSelectedPointIndex;
        auto p1 = fSegStructMapRef[fSelectedSegID].fIntersectionCurve.GetPoint(fSelectedPointIndex);
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
        // qDebug() << "v2: " << v2 << " v: " << v << " p2: " << p2 << " p1: " << p1[0] << " " << p1[1];
        v2 += p2;
        // qDebug() << "v2: " << v2 << " scrollPositionModifier: " << scrollPositionModifier;
        ScrollToCenter(v2 * fScaleFactor);
    }
}

// Handle mouse press event
void CVolumeViewerWithCurve::mousePressEvent(QMouseEvent* event)
{
    // Check if back or forward button was pressed
    if (event->buttons() & Qt::BackButton || event->buttons() & Qt::ForwardButton) {
        lastPressedButton = event->button();
        scrollPositionModifier = cv::Vec2f(0.0, 0.0);
        timer->start(fwdBackMsJump); // start timer with millisec delay value from settings
        return;
    }

    if (lastPressedButton & Qt::NoButton) {
        return;
    }

    // Return if not left or right click
    if (!(event->buttons() & Qt::RightButton) && !(event->buttons() & Qt::LeftButton) && !fIsMousePressed) {
        return;
    }

    if((event->buttons() & Qt::LeftButton) || fIsMousePressed) {
        // Get the mouse position in widget coordinates
        cv::Vec2f aWidgetLoc, aImgLoc, res;
        // For some reason the mouse position in the relase event are different and slightly off.
        // So we need to use the values from the press event here.
        aWidgetLoc[0] = event->position().x();
        aWidgetLoc[1] = event->position().y();

        // Convert to image coordinates
        WidgetLoc2ImgLoc(aWidgetLoc, aImgLoc);

        // Update the last tracked position
        fLastPos.setX(aImgLoc[0]);
        fLastPos.setY(aImgLoc[1]);

        // Handle draw and edit
        if (fViewState == EViewState::ViewStateDraw && (event->buttons() & Qt::LeftButton)) {
            // With left click, add control points to the curve
            fControlPoints.push_back(aImgLoc);
            UpdateSplineCurve();
            UpdateView();
                
        } else if (fViewState == EViewState::ViewStateEdit && ((event->buttons() & Qt::LeftButton) || fIsMousePressed)) {
            fIsMousePressed = true;        

            if(fImageIndex != sliceIndexToolStart) {
                SendSignalStatusMessageAvailable(tr("Tool was started for slice %1. No other slices can be edited right now.").arg(QString::number(sliceIndexToolStart)), 3000);
                return;
            }
            
            // If we have points, select the one that was clicked.
            auto res = SelectPointOnCurves(aImgLoc, true);
            fSelectedPointIndex = res.first;
            fSelectedSegID = res.second;

            // Set the curve to the one that was clicked
            fSegStructMapRef[fSelectedSegID].fIntersectionCurve.setLastState();

            // Set fLastPos to the position of the selected point
            if (fSelectedPointIndex >= 0) {
                setCursor(Qt::PointingHandCursor);

                fLastPos.setX(fSegStructMapRef[fSelectedSegID].fIntersectionCurve.GetPoint(fSelectedPointIndex)[0]);
                fLastPos.setY(fSegStructMapRef[fSelectedSegID].fIntersectionCurve.GetPoint(fSelectedPointIndex)[1]);
                
                // Mouse move event to update the line
                mouseMoveEvent(event);

                // qDebug() << "mousePressEvent: selected point index: " << fSelectedPointIndex;
            }
        }
    } else if (event->buttons() & Qt::RightButton) {
        // Attempt to start the panning. We only consider us in panning mode, if in the mouse move event
        // we actually detect movement.
        
        rightPressed = true;
        wantsPanning = true;
        isPanning = false;
        panStartX = event->position().x();
        panStartY = event->position().y();
    }    
}

// Handle mouse move event, currently only when we're editing
void CVolumeViewerWithCurve::mouseMoveEvent(QMouseEvent* event)
{
    // If we have an active last pressed button from the backwards/forwards move feature,
    // we cannot do any panning at the same time, nor do we want to move any paths.
    if(lastPressedButton) {
        return;
    }
        
    if (!wantsPanning && !rightPressed & !fIsMousePressed) {
        return;
    }

    // Update the curve if we have a selected point
    if (fSelectedPointIndex != -1) {
        // Get the mouse position in widget coordinates
        cv::Vec2f aWidgetLoc, aImgLoc;
        aWidgetLoc[0] = event->position().x();
        aWidgetLoc[1] = event->position().y();

        // Convert to image coordinates and get delta of change
        WidgetLoc2ImgLoc(aWidgetLoc, aImgLoc);
        Vec2<double> aDelta;
        aDelta[0] = aImgLoc[0] - fLastPos.x();
        aDelta[1] = aImgLoc[1] - fLastPos.y();

        fSegStructMapRef[fSelectedSegID].fIntersectionCurve.SetPointByDifference(
            fSelectedPointIndex, aDelta, CosineImpactFunc, fImpactRange);
        fVertexIsChanged = true;

        UpdateView();

    } else if (wantsPanning && rightPressed){
        // We potentially want to start panning, and now check if the mouse actually moved    
        if(event->position().x() != panStartX || event->position().y() - panStartY)
        {
            isPanning = true;
            setCursor(Qt::ClosedHandCursor);
            fGraphicsView->horizontalScrollBar()->setValue(fGraphicsView->horizontalScrollBar()->value() - 2 * (event->position().x() - panStartX));
            fGraphicsView->verticalScrollBar()->setValue(fGraphicsView->verticalScrollBar()->value() - 2 * (event->position().y() - panStartY));
            panStartX = event->position().x();
            panStartY = event->position().y();
        } else {
            wantsPanning = false;
        }
    }    
}

// Handle mouse release event
void CVolumeViewerWithCurve::mouseReleaseEvent(QMouseEvent* event)
{
    if (((lastPressedButton & Qt::BackButton) && !(event->buttons() & Qt::BackButton)) || ((lastPressedButton & Qt::ForwardButton) && !(event->buttons() & Qt::ForwardButton))) {
        timer->stop();
        lastPressedButton = Qt::NoButton;  // unset the last pressed button
    }

    if (!(event->buttons() & Qt::RightButton) && !(event->buttons() & Qt::LeftButton)) {
        fIsMousePressed = false;
    }

    // End panning
    if(event->button() == Qt::RightButton) {
        isPanning = wantsPanning = rightPressed = false;
        setCursor(Qt::ArrowCursor);
        event->accept();
        return;
    }

    if (fIntersectionCurveRef != nullptr && fVertexIsChanged) {

        // update the point positions in the path point cloud
        SendSignalPathChanged();

        fVertexIsChanged = false;
        fSelectedPointIndex = -1;
    }

    setCursor(Qt::ArrowCursor);
}

// capture mouse release
bool CVolumeViewerWithCurve::eventFilter(QObject* watched, QEvent* event) 
{
    // check for mouse release generic
    if (event->type() == QEvent::MouseButtonRelease) {
        QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
        mouseReleaseEvent(mouseEvent);
        event->accept();
        return true;
    }

    if (event->type() == QEvent::MouseMove) {
        QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
        
        // Transform the global coordinates to local coordinates
        QPointF localPoint = this->mapFromGlobal(mouseEvent->globalPosition());
        
        // Create a new QMouseEvent with local coordinates
        QMouseEvent localMouseEvent(QEvent::MouseMove,
                                    localPoint,
                                    mouseEvent->globalPosition(),
                                    mouseEvent->button(),
                                    mouseEvent->buttons(),
                                    mouseEvent->modifiers());
        
        // Manually call your mouseMoveEvent function
        mouseMoveEvent(&localMouseEvent);
        
        event->accept();
        return true;
    }

    // also call parent class implementation
    return CVolumeViewer::eventFilter(watched, event);
}

// Handle paint event
void CVolumeViewerWithCurve::paintEvent(QPaintEvent* /*event*/) {}

void CVolumeViewerWithCurve::toggleShowCurveBox()
{
    bool currentState = fShowCurveBox->isChecked();
    fShowCurveBox->setChecked(!currentState);
    UpdateView();
}

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

void CVolumeViewerWithCurve::WidgetLoc2ImgLoc(
    const cv::Vec2f& nWidgetLoc, cv::Vec2f& nImgLoc)
{
    // Step 1: Convert widget coordinates to scene coordinates
    QPointF widgetPoint(nWidgetLoc[0] - (fGraphicsView->pos()).x() - 2, nWidgetLoc[1] - (fGraphicsView->pos()).y() - 2);
    //widgetPoint = widgetPoint - fGraphicsView->pos();
        
    QPointF scenePoint = fGraphicsView->mapToScene(widgetPoint.toPoint());
  
    // Step 2: Convert scene coordinates to item coordinates
    QPointF itemPoint = fBaseImageItem->mapFromScene(scenePoint);
  
    nImgLoc[0] = static_cast<float>(itemPoint.x());
    nImgLoc[1] = static_cast<float>(itemPoint.y());
}

// Select point on curves
std::pair<int, std::string> CVolumeViewerWithCurve::SelectPointOnCurves(
    const cv::Vec2f& nPt, bool snapMode, bool selectGlobally)
{
    const double DIST_THRESHOLD = 1.5 * fScaleFactor;

    double minDistance = std::numeric_limits<double>::max();
    std::string best_id = "";
    int closestPointIndex = -1;

    for (auto& seg : fSegStructMapRef) {
        auto& segStruct = seg.second;
        if ((!segStruct.compute && !selectGlobally) || !segStruct.display || segStruct.fIntersectionCurve.GetPointsNum()==0) {
            continue;
        }
        // Only select points on curve that is selected
        if (selectGlobally && !segStruct.highlighted) {
            continue;
        }
        for (size_t i = 0; i < segStruct.fIntersectionCurve.GetPointsNum(); ++i) {
            double currentDistance = Norm<double>(Vec2<double>(
                segStruct.fIntersectionCurve.GetPoint(i)[0] - nPt[0],
                segStruct.fIntersectionCurve.GetPoint(i)[1] - nPt[1]));

            if (currentDistance < minDistance) {
                minDistance = currentDistance;
                closestPointIndex = i;
                best_id = seg.first;
            }
        }
    }
    // qDebug() << "minDistance: " << minDistance << " closestPointIndex: " << closestPointIndex << " best_id: " << best_id.c_str();
    if (selectGlobally || (snapMode && minDistance < 100.0*DIST_THRESHOLD) || minDistance < DIST_THRESHOLD) {
        // qDebug() << "returning closestPointIndex: " << closestPointIndex << " best_id: " << best_id.c_str();
        return std::make_pair(closestPointIndex, best_id);
    } else {
        // qDebug() << "returning -1";
        return std::make_pair(-1, "");
    }
}

// Draw intersection curve on the slice
void CVolumeViewerWithCurve::DrawIntersectionCurve(QGraphicsScene* scene) {
    for (auto& seg : fSegStructMapRef) {
        auto& segStruct = seg.second;
        int r{0}, g{0}, b{0};
        if (segStruct.highlighted && segStruct.compute) {
            colorSelectorHighlight->color().getRgb(&r, &g, &b);
        }
        else if (segStruct.compute) {
            colorSelectorCompute->color().getRgb(&r, &g, &b);
        }
        else if (segStruct.display && segStruct.highlighted) {
            // Mix the colors to show Highlight and Display without Compute
            r = (colorSelectorHighlight->color().red() + colorSelector->color().red()) / 2;
            g = (colorSelectorHighlight->color().green() + colorSelector->color().green()) / 2;
            b = (colorSelectorHighlight->color().blue() + colorSelector->color().blue()) / 2;
        }
        else {
            colorSelector->color().getRgb(&r, &g, &b);
        }
        if (!scene || !segStruct.display || segStruct.fIntersectionCurve.GetPointsNum()==0 || !colorSelector) {
            // qDebug() << "DrawIntersectionCurve: early exit. Scene " << scene << " segStruct.display " << segStruct.display << " segStruct.fIntersectionCurve.GetPointsNum() " << segStruct.fIntersectionCurve.GetPointsNum() << " colorSelector " << colorSelector;
            // qDebug() << "segStruct id " << segStruct.fSegmentationId.c_str();
            continue;  // Early continue if either object is null or the list is empty
        }
        // qDebug() << "Drawing segStruct id " << segStruct.fSegmentationId.c_str() << " with num points " << segStruct.fIntersectionCurve.GetPointsNum();
        int pointsNum = segStruct.fIntersectionCurve.GetPointsNum();

        for (int i = 0; i < pointsNum; ++i) {
            // Create new ellipse points
            auto p0 = segStruct.fIntersectionCurve.GetPoint(i)[0] - 0.5;
            auto p1 = segStruct.fIntersectionCurve.GetPoint(i)[1] - 0.5;
            QGraphicsEllipseItem* newEllipse = scene->addEllipse(p0, p1, 2, 2, QPen(QColor(r, g, b)), QBrush(QColor(r, g, b)));
            newEllipse->setVisible(true);
        }
    }
}

void CVolumeViewerWithCurve::DrawControlPoints(QGraphicsScene* scene) {
    int r{0}, g{0}, b{0};
    colorSelector->color().getRgb(&r, &g, &b);
    if (!scene || fControlPoints.empty() || !colorSelector) {
        return;  // Early exit if either object is null or the list is empty
    }

    for (int i = 0; i < fControlPoints.size(); ++i) {
        // Create new ellipse points
        auto p0 = fControlPoints[i][0] - 0.5;
        auto p1 = fControlPoints[i][1] - 0.5;
        QGraphicsEllipseItem* newEllipse = scene->addEllipse(p0, p1, 2, 2, QPen(QColor(r, g, b)), QBrush(QColor(r, g, b)));
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
        fImgQImage != nullptr && (fViewState == EViewState::ViewStateIdle || fViewState == EViewState::ViewStateEdit));
    fPrevBtn->setEnabled(
        fImgQImage != nullptr && (fViewState == EViewState::ViewStateIdle || fViewState == EViewState::ViewStateEdit));
    fImageIndexEdit->setEnabled(fViewState == EViewState::ViewStateIdle || fViewState == EViewState::ViewStateEdit);
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
