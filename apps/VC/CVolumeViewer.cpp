// CVolumeViewer.cpp
// Chao Du 2015 April
#include "CVolumeViewer.hpp"
#include "HBase.hpp"

using namespace ChaoVis;
using qga = QGuiApplication;

#define BGND_RECT_MARGIN 8
#define DEFAULT_TEXT_COLOR QColor(255, 255, 120)
#define ZOOM_FACTOR 1.15

// Constructor
CVolumeViewerView::CVolumeViewerView(QWidget* parent)
: QGraphicsView(parent)
{
    timerTextAboveCursor = new QTimer(this);
    connect(timerTextAboveCursor, &QTimer::timeout, this, &CVolumeViewerView::hideTextAboveCursor);
    timerTextAboveCursor->setSingleShot(true);
}

void CVolumeViewerView::setup()
{
    textAboveCursor = new QGraphicsTextItem("", 0);
    textAboveCursor->setFlag(QGraphicsItem::ItemIgnoresTransformations);
    textAboveCursor->setZValue(100);
    textAboveCursor->setVisible(false);
    textAboveCursor->setDefaultTextColor(DEFAULT_TEXT_COLOR);
    scene()->addItem(textAboveCursor);

    QFont f;
    f.setPointSize(f.pointSize() + 2);
    textAboveCursor->setFont(f);

    backgroundBehindText = new QGraphicsRectItem();
    backgroundBehindText->setFlag(QGraphicsItem::ItemIgnoresTransformations);
    backgroundBehindText->setPen(Qt::NoPen);
    backgroundBehindText->setZValue(99);
    scene()->addItem(backgroundBehindText);
}

void CVolumeViewerView::keyPressEvent(QKeyEvent* event)
{
    if (event->key() == Qt::Key_W) {
        rangeKeyPressed = true;
        event->accept();
    } else if (event->key() == Qt::Key_R) {
        curvePanKeyPressed = true;
        event->accept();
    } else if (event->key() == Qt::Key_S) {
        rotateKeyPressed = true;
        event->accept();
    }
}

void CVolumeViewerView::keyReleaseEvent(QKeyEvent* event)
{
    if (event->key() == Qt::Key_W)  {
        rangeKeyPressed = false;
        event->accept();
    } else if (event->key() == Qt::Key_R) {
        curvePanKeyPressed = false;
        event->accept();
    } else if (event->key() == Qt::Key_S) {
        rotateKeyPressed = false;
        event->accept();
    }
}

void CVolumeViewerView::showTextAboveCursor(const QString& value, const QString& label, const QColor& color)
{
    // Without this check, when you start VC with auto-load the initial slice will not be in the center of
    // volume viewer, because during loading the initilization of the impact range slider and its callback slots
    // will already move the position/scrollbars of the viewer and therefore the image is no longer centered.
    if (!isVisible()) {
        return;
    }

    timerTextAboveCursor->start(150);

    QFontMetrics fm(textAboveCursor->font());
    QPointF p = mapToScene(mapFromGlobal(QPoint(QCursor::pos().x() + 10, QCursor::pos().y())));

    textAboveCursor->setVisible(true);
    textAboveCursor->setHtml("<b>" + value + "</b><br>" + label);
    textAboveCursor->setPos(p);
    textAboveCursor->setDefaultTextColor(color);

    backgroundBehindText->setVisible(true);
    backgroundBehindText->setPos(p);
    backgroundBehindText->setRect(0, 0, fm.horizontalAdvance((label.isEmpty() ? value : label)) + BGND_RECT_MARGIN, fm.height() * (label.isEmpty() ? 1 : 2) + BGND_RECT_MARGIN);
    backgroundBehindText->setBrush(QBrush(QColor(
        (2 * 125 + color.red())   / 3,
        (2 * 125 + color.green()) / 3,
        (2 * 125 + color.blue())  / 3,
    200)));
}

void CVolumeViewerView::hideTextAboveCursor()
{
    textAboveCursor->setVisible(false);
    backgroundBehindText->setVisible(false);
}

void CVolumeViewerView::showCurrentImpactRange(int range)
{
    showTextAboveCursor(QString::number(range), "", QColor(255, 120, 110)); // tr("Impact Range")
}

void CVolumeViewerView::showCurrentScanRange(int range)
{
    showTextAboveCursor(QString::number(range), "", QColor(160, 180, 255)); // tr("Scan Range")
}

void CVolumeViewerView::showCurrentSliceIndex(int slice, bool highlight)
{
    showTextAboveCursor(QString::number(slice), "", (highlight ? QColor(255, 50, 20) : QColor(255, 220, 30))); // tr("Slice")
}

// Constructor
CVolumeViewer::CVolumeViewer(QWidget* parent)
    : QWidget(parent)
    , fCanvas(nullptr)
    , fScrollArea(nullptr)
    , fGraphicsView(nullptr)
    , fZoomInBtn(nullptr)
    , fZoomOutBtn(nullptr)
    , fResetBtn(nullptr)
    , fNextBtn(nullptr)
    , fPrevBtn(nullptr)
    , fViewState(EViewState::ViewStateIdle)
    , fImgQImage(nullptr)
    , fBaseImageItem(nullptr)
    , fScaleFactor(1.0)
    , fImageIndex(0)
    , fScanRange(1)
{
    // buttons
    fZoomInBtn = new QPushButton(tr("Zoom In"), this);
    fZoomOutBtn = new QPushButton(tr("Zoom Out"), this);
    fResetBtn = new QPushButton(tr("Reset"), this);
    fNextBtn = new QPushButton(tr("Next Slice"), this);
    fPrevBtn = new QPushButton(tr("Previous Slice"), this);

    // slice index edit
    fImageIndexEdit = new QSpinBox(this);
    fImageIndexEdit->setMinimum(0);
    fImageIndexEdit->setEnabled(true);
    fImageIndexEdit->setMinimumWidth(100);
    connect(
        fImageIndexEdit, SIGNAL(editingFinished()), this,
        SLOT(OnImageIndexEditTextChanged()));

    fBaseImageItem = nullptr;

    // Create graphics view
    fGraphicsView = new CVolumeViewerView(this);
    fGraphicsView->setRenderHint(QPainter::Antialiasing);
    setFocusProxy(fGraphicsView);

    // Create graphics scene
    fScene = new QGraphicsScene(this);

    // Set the scene
    fGraphicsView->setScene(fScene);
    fGraphicsView->setup();

    fGraphicsView->viewport()->installEventFilter(this);

    fButtonsLayout = new QHBoxLayout;
    fButtonsLayout->addWidget(fZoomInBtn);
    fButtonsLayout->addWidget(fZoomOutBtn);
    fButtonsLayout->addWidget(fResetBtn);
    fButtonsLayout->addWidget(fPrevBtn);
    fButtonsLayout->addWidget(fNextBtn);
    fButtonsLayout->addWidget(fImageIndexEdit);
    // Add some space between the slice spin box and the curve tools (color, checkboxes, ...)
    fButtonsLayout->addSpacerItem(new QSpacerItem(1, 0, QSizePolicy::Expanding, QSizePolicy::Fixed));

    connect(fZoomInBtn, SIGNAL(clicked()), this, SLOT(OnZoomInClicked()));
    connect(fZoomOutBtn, SIGNAL(clicked()), this, SLOT(OnZoomOutClicked()));
    connect(fResetBtn, SIGNAL(clicked()), this, SLOT(OnResetClicked()));
    connect(fNextBtn, SIGNAL(clicked()), this, SLOT(OnNextClicked()));
    connect(fPrevBtn, SIGNAL(clicked()), this, SLOT(OnPrevClicked()));

    QSettings settings("VC.ini", QSettings::IniFormat);
    fCenterOnZoomEnabled = settings.value("viewer/center_on_zoom", false).toInt() != 0;

    QVBoxLayout* aWidgetLayout = new QVBoxLayout;
    aWidgetLayout->addWidget(fGraphicsView);
    aWidgetLayout->addLayout(fButtonsLayout);

    setLayout(aWidgetLayout);

    UpdateButtons();
}

// Destructor
CVolumeViewer::~CVolumeViewer(void)
{
    deleteNULL(fGraphicsView);
    deleteNULL(fScene);
    deleteNULL(fScrollArea);
    deleteNULL(fZoomInBtn);
    deleteNULL(fZoomOutBtn);
    deleteNULL(fResetBtn);
    deleteNULL(fPrevBtn);
    deleteNULL(fNextBtn);
    deleteNULL(fImageIndexEdit);
}

void CVolumeViewer::setButtonsEnabled(bool state)
{
    fZoomOutBtn->setEnabled(state);
    fZoomInBtn->setEnabled(state);
    fPrevBtn->setEnabled(state);
    fNextBtn->setEnabled(state);
    fImageIndexEdit->setEnabled(state);
}

void CVolumeViewer::SetImage(const QImage& nSrc)
{
    if (fImgQImage == nullptr) {
        fImgQImage = new QImage(nSrc);
    } else {
        *fImgQImage = nSrc;
    }

    // Create a QPixmap from the QImage
    QPixmap pixmap = QPixmap::fromImage(*fImgQImage);

    // Add the QPixmap to the scene as a QGraphicsPixmapItem
    if (fBaseImageItem) {
        // If the item already exists, remove it from the scene
        fScene->removeItem(fBaseImageItem);
        delete fBaseImageItem; // Delete the old item
    }
    fBaseImageItem = fScene->addPixmap(pixmap);

    UpdateButtons();
    update();
}

void CVolumeViewer::setNumSlices(int num)
{
    fImageIndexEdit->setMaximum(num);
}

bool CVolumeViewer::eventFilter(QObject* watched, QEvent* event)
{
    // Wheel events
    if (watched == fGraphicsView || (fGraphicsView && watched == fGraphicsView->viewport()) && event->type() == QEvent::Wheel) {

        QWheelEvent* wheelEvent = static_cast<QWheelEvent*>(event);

        // Range key pressed
        if (fGraphicsView->isRangeKeyPressed()) {
            int numDegrees = wheelEvent->angleDelta().y() / 8;

            if (numDegrees > 0) {
                SendSignalImpactRangeUp();
            } else if (numDegrees < 0) {
                SendSignalImpactRangeDown();
            }

            return true;
        }
        // Ctrl = Zoom in/out
        else if (QApplication::keyboardModifiers() == Qt::ControlModifier) {
            int numDegrees = wheelEvent->angleDelta().y() / 8;

            if (numDegrees > 0) {
                OnZoomInClicked();
            } else if (numDegrees < 0) {
                OnZoomOutClicked();
            }

            if (fCenterOnZoomEnabled) {
                CenterOn(fGraphicsView->mapToScene(wheelEvent->position().toPoint()));
            }

            return true;
        }
        // Shift = Scan through slices
        else if (QApplication::keyboardModifiers() == Qt::ShiftModifier) {
            int numDegrees = wheelEvent->angleDelta().y() / 8;

            if (numDegrees > 0) {
                SendSignalOnNextSliceShift(fScanRange);
                fGraphicsView->showCurrentSliceIndex(fImageIndex, (GetViewState() == EViewState::ViewStateEdit && fImageIndex == sliceIndexToolStart));
            } else if (numDegrees < 0) {
                SendSignalOnPrevSliceShift(fScanRange);
                fGraphicsView->showCurrentSliceIndex(fImageIndex, (GetViewState() == EViewState::ViewStateEdit && fImageIndex == sliceIndexToolStart));
            }
            return true;
        }
        // Rotate key pressed
        else if (fGraphicsView->isRotateKyPressed()) {
            int delta = wheelEvent->angleDelta().y() / 22;
            fGraphicsView->rotate(delta);
            fGraphicsView->updateCurrentRotation(delta);
            return true;
        }
    }
    return QWidget::eventFilter(watched, event);
}

void CVolumeViewer::ScaleImage(double nFactor)
{
    fScaleFactor *= nFactor;
    fGraphicsView->scale(nFactor, nFactor);

    UpdateButtons();
}

void CVolumeViewer::CenterOn(const QPointF& point)
{
    fGraphicsView->centerOn(point);
}

void CVolumeViewer::ResetRotation()
{
    auto current = fGraphicsView->getCurrentRotation();
    fGraphicsView->rotate(-current);
    fGraphicsView->updateCurrentRotation(-current);
}

// Handle zoom in click
void CVolumeViewer::OnZoomInClicked(void)
{
    if (fZoomInBtn->isEnabled()) {
        ScaleImage(ZOOM_FACTOR);
    }
}

// Handle zoom out click
void CVolumeViewer::OnZoomOutClicked(void)
{
    if (fZoomOutBtn->isEnabled()) {
        ScaleImage(1 / ZOOM_FACTOR);
    }
}

void CVolumeViewer::OnResetClicked(void)
{
    fGraphicsView->resetTransform();
    fScaleFactor = 1.0;

    UpdateButtons();
}

// Handle next image click
void CVolumeViewer::OnNextClicked(void)
{
    // send signal to controller (MVC) in order to update the content
    if (fNextBtn->isEnabled()) {
        // If the signal sender is the button, we check for Shift modifier for bigger jumps
        if (sender() == fNextBtn)
            SendSignalOnNextSliceShift(qga::keyboardModifiers() == Qt::ShiftModifier ? 10 : 1);
        else
            SendSignalOnNextSliceShift(1);
    }
}

// Handle previous image click
void CVolumeViewer::OnPrevClicked(void)
{
    // send signal to controller (MVC) in order to update the content
    if (fPrevBtn->isEnabled()) {
        // If the signal sender is the button, we check for Shift modifier for bigger jumps
        if (sender() == fPrevBtn)
            SendSignalOnPrevSliceShift(qga::keyboardModifiers() == Qt::ShiftModifier ? 10 : 1);
        else
            SendSignalOnPrevSliceShift(1);
    }
}

// Handle image index change
void CVolumeViewer::OnImageIndexEditTextChanged(void)
{
    // send signal to controller in order to update the content
    SendSignalOnLoadAnyImage(fImageIndexEdit->value());
}

// Update the status of the buttons
void CVolumeViewer::UpdateButtons(void)
{
    fZoomInBtn->setEnabled(fImgQImage != nullptr && fScaleFactor < 10.0);
    fZoomOutBtn->setEnabled(fImgQImage != nullptr && fScaleFactor > 0.05);
    fResetBtn->setEnabled(
        fImgQImage != nullptr && fabs(fScaleFactor - 1.0) > 1e-6);
    fNextBtn->setEnabled(fImgQImage != nullptr);
    fPrevBtn->setEnabled(fImgQImage != nullptr);
}