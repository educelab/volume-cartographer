// CVolumeViewer.cpp
// Chao Du 2015 April
#include "CVolumeViewer.hpp"
#include "HBase.hpp"

using namespace ChaoVis;
using qga = QGuiApplication;

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
    , fImgQImage(nullptr)
    , fBaseImageItem(nullptr)
    , fScaleFactor(1.0)
    , fImageIndex(0)
{
    // buttons
    fZoomInBtn = new QPushButton(tr("Zoom In"), this);
    fZoomOutBtn = new QPushButton(tr("Zoom Out"), this);
    fResetBtn = new QPushButton(tr("Reset"), this);
    fNextBtn = new QPushButton(tr("Next Slice"), this);
    fPrevBtn = new QPushButton(tr("Previous Slice"), this);

    // text edit
    fImageIndexEdit = new QSpinBox(this);
    fImageIndexEdit->setMinimum(0);
    fImageIndexEdit->setEnabled(true);
    connect(
        fImageIndexEdit, SIGNAL(valueChanged(int)), this,
        SLOT(OnImageIndexEditTextChanged(int)));

    fBaseImageItem = new QGraphicsPixmapItem();

    // Create graphics view
    fGraphicsView = new QGraphicsView(this);
    fGraphicsView->setRenderHint(QPainter::Antialiasing);
    
    // Create graphics scene
    fScene = new QGraphicsScene(this);

    // Set the scene
    fGraphicsView->setScene(fScene);

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
    if(fBaseImageItem) {
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
    if ((watched == fGraphicsView || (fGraphicsView && watched == fGraphicsView->viewport())) && event->type() == QEvent::Wheel) {
        QWheelEvent* wheelEvent = static_cast<QWheelEvent*>(event);
        if(QApplication::keyboardModifiers() == Qt::ControlModifier) {
            int numDegrees = wheelEvent->angleDelta().y() / 8;

            if (numDegrees > 0) {
                OnZoomInClicked();
            } else if (numDegrees < 0) {
                OnZoomOutClicked();
            }
            return true;
        } else if(QApplication::keyboardModifiers() == Qt::ShiftModifier) {
            int numDegrees = wheelEvent->angleDelta().y() / 8;

            if (numDegrees > 0) {
                OnNextClicked();
            } else if (numDegrees < 0) {
                OnPrevClicked();
            }
            return true;
        }
    }
    return QWidget::eventFilter(watched, event);
}

// Handle paint event
void CVolumeViewer::paintEvent(QPaintEvent* /*event*/)
{
    // REVISIT - FILL ME HERE
}

void CVolumeViewer::ScaleImage(double nFactor)
{
    fScaleFactor *= nFactor;
    fGraphicsView->scale(nFactor, nFactor);

    UpdateButtons();
}

// Handle zoom in click
void CVolumeViewer::OnZoomInClicked(void)
{
    if (fZoomInBtn->isEnabled()) {
        ScaleImage(1.25);
    }
}

// Handle zoom out click
void CVolumeViewer::OnZoomOutClicked(void)
{
    if (fZoomOutBtn->isEnabled()) {
        ScaleImage(0.8);
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
        // If the signal sender is the button, check for the Shift modifier in order to jump some slices.
        // For all other sources, such as the mouse wheel this jump should not happen.
        SendSignalOnNextClicked(sender() == fNextBtn && qga::keyboardModifiers() == Qt::ShiftModifier ? true : false);
    }
}

// Handle previous image click
void CVolumeViewer::OnPrevClicked(void)
{
    // send signal to controller (MVC) in order to update the content
    if (fPrevBtn->isEnabled()) {
        // If the signal sender is the button, check for the Shift modifier in order to jump some slices.
        // For all other sources, such as the mouse wheel this jump should not happen.
        SendSignalOnPrevClicked(sender() == fPrevBtn && qga::keyboardModifiers() == Qt::ShiftModifier ? true : false);
    }
}

// Handle image index change
void CVolumeViewer::OnImageIndexEditTextChanged(int index)
{
    // send signal to controller in order to update the content
    SendSignalOnLoadAnyImage(index);
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

// Adjust scroll bar of scroll area
void CVolumeViewer::AdjustScrollBar(QScrollBar* nScrollBar, double nFactor)
{
    nScrollBar->setValue(
        int(nFactor * nScrollBar->value() +
            ((nFactor - 1) * nScrollBar->pageStep() / 2)));
}

cv::Vec2f CVolumeViewer::CleanScrollPosition(cv::Vec2f pos) const
{
    int x = pos[0];
    int y = pos[1];

    // Get the size of the QGraphicsView viewport
    int viewportWidth = fGraphicsView->viewport()->width();
    int viewportHeight = fGraphicsView->viewport()->height();

    // Calculate the position of the scroll bars
    int horizontalPos = x - viewportWidth / 2;
    int verticalPos = y - viewportHeight / 2;

    // Check and respect horizontal boundaries
    if(horizontalPos < fGraphicsView->horizontalScrollBar()->minimum())
        horizontalPos = fGraphicsView->horizontalScrollBar()->minimum();
    else if(horizontalPos > fGraphicsView->horizontalScrollBar()->maximum())
        horizontalPos = fGraphicsView->horizontalScrollBar()->maximum();

    // Check and respect vertical boundaries
    if(verticalPos < fGraphicsView->verticalScrollBar()->minimum())
        verticalPos = fGraphicsView->verticalScrollBar()->minimum();
    else if(verticalPos > fGraphicsView->verticalScrollBar()->maximum())
        verticalPos = fGraphicsView->verticalScrollBar()->maximum();

    return cv::Vec2f(horizontalPos + viewportWidth / 2, verticalPos + viewportHeight / 2);
}

void CVolumeViewer::ScrollToCenter(cv::Vec2f pos)
{    
    pos = CleanScrollPosition(pos);

    // Get the size of the QGraphicsView viewport
    int viewportWidth = fGraphicsView->viewport()->width();
    int viewportHeight = fGraphicsView->viewport()->height();

    // Calculate the position of the scroll bars
    int horizontalPos = pos[0] - viewportWidth / 2;
    int verticalPos = pos[1] - viewportHeight / 2;

    // Set the scroll bar positions
    fGraphicsView->horizontalScrollBar()->setValue(horizontalPos);
    fGraphicsView->verticalScrollBar()->setValue(verticalPos);
}

cv::Vec2f CVolumeViewer::GetScrollPosition() const
{
    // Get the positions of the scroll bars
    float horizontalPos = static_cast<float>(fGraphicsView->horizontalScrollBar()->value() + fGraphicsView->viewport()->width() / 2);
    float verticalPos = static_cast<float>(fGraphicsView->verticalScrollBar()->value() + fGraphicsView->viewport()->height() / 2);

    // Return as cv::Vec2f
    return cv::Vec2f(horizontalPos, verticalPos);
}
