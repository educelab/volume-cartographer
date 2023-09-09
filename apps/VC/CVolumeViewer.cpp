// CVolumeViewer.cpp
// Chao Du 2015 April
#include "CVolumeViewer.hpp"
#include "HBase.hpp"

using namespace ChaoVis;

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
    fImageIndexEdit = new CSimpleNumEditBox(this);
    fImageIndexEdit->setEnabled(true);
    connect(
        fImageIndexEdit, SIGNAL(SendSignalOnTextChanged()), this,
        SLOT(OnImageIndexEditTextChanged()));

    fBaseImageItem = new QGraphicsPixmapItem();

    // Create graphics view
    fGraphicsView = new QGraphicsView(this);
    fGraphicsView->setRenderHint(QPainter::Antialiasing);
    
    // Create graphics scene
    fScene = new QGraphicsScene(this);

    // Set the scene
    fGraphicsView->setScene(fScene);

    // fImgQImage = new QImage();

    // // create image label
    // fCanvas = new QLabel;
    // fCanvas->setBackgroundRole(QPalette::Base);
    // fCanvas->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    // fCanvas->setScaledContents(true);

    // // create scroll area
    // fScrollArea = new QScrollArea;
    // fScrollArea->setBackgroundRole(QPalette::Dark);
    // fScrollArea->setWidget(fGraphicsView);
    // // Install the event filter
    // fScrollArea->viewport()->installEventFilter(this);
    fGraphicsView->viewport()->installEventFilter(this);

    fButtonsLayout = new QHBoxLayout;
    fButtonsLayout->addWidget(fZoomInBtn);
    fButtonsLayout->addWidget(fZoomOutBtn);
    fButtonsLayout->addWidget(fResetBtn);
    fButtonsLayout->addWidget(fPrevBtn);
    fButtonsLayout->addWidget(fNextBtn);
    fButtonsLayout->addWidget(fImageIndexEdit);

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
    // deleteNULL(fCanvas);
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

// // Set image
// void CVolumeViewer::SetImage(const QImage& nSrc)
// {
//     if (fImgQImage == nullptr) {
//         fImgQImage = new QImage(nSrc);
//     } else {
//         *fImgQImage = nSrc;
//     }

//     fCanvas->setPixmap(QPixmap::fromImage(*fImgQImage));
//     fCanvas->resize(fScaleFactor * fCanvas->pixmap(Qt::ReturnByValue).size());

//     UpdateButtons();
//     update();
// }

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

    // Optionally, set the scene size to match the image size
    // fScene->setSceneRect(pixmap.rect());

    // Apply scaling if necessary
    // fBaseImageItem->setScale(fScaleFactor);  // Be cautious about scaling a QGraphicsPixmapItem

    UpdateButtons();
    update();
}



bool CVolumeViewer::eventFilter(QObject* watched, QEvent* event)
{
    if ((watched == fGraphicsView || watched == fGraphicsView->viewport()) && event->type() == QEvent::Wheel) {
        QWheelEvent* wheelEvent = static_cast<QWheelEvent*>(event);
        if(QApplication::keyboardModifiers() == Qt::ShiftModifier) {
            int numDegrees = wheelEvent->angleDelta().y() / 8;

            if (numDegrees > 0) {
                OnZoomInClicked();
            } else if (numDegrees < 0) {
                OnZoomOutClicked();
            }
            // event->accept();
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

// // Scale image
// void CVolumeViewer::ScaleImage(double nFactor)
// {
//     Q_ASSERT(!fCanvas->pixmap(Qt::ReturnByValue).isNull());

//     fScaleFactor *= nFactor;
//     fCanvas->resize(fScaleFactor * fCanvas->pixmap(Qt::ReturnByValue).size());

//     AdjustScrollBar(fScrollArea->horizontalScrollBar(), nFactor);
//     AdjustScrollBar(fScrollArea->verticalScrollBar(), nFactor);

//     UpdateButtons();
// }

void CVolumeViewer::ScaleImage(double nFactor)
{
    fScaleFactor *= nFactor;
    fGraphicsView->scale(nFactor, nFactor);

    //AdjustScrollBar(fGraphicsView->horizontalScrollBar(), nFactor);
    //AdjustScrollBar(fGraphicsView->verticalScrollBar(), nFactor);

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

// // Handle reset click
// void CVolumeViewer::OnResetClicked(void)
// {
//     fCanvas->adjustSize();
//     fScaleFactor = 1.0;

//     UpdateButtons();
// }

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
        SendSignalOnNextClicked();
    }
}

// Handle previous image click
void CVolumeViewer::OnPrevClicked(void)
{
    // send signal to controller (MVC ) in order to update the content
    if (fPrevBtn->isEnabled()) {
        SendSignalOnPrevClicked();
    }
}

// Handle image index change
void CVolumeViewer::OnImageIndexEditTextChanged(void)
{
    // send signal to controller in order to update the content
    SendSignalOnLoadAnyImage(fImageIndexEdit->GetImageIndex());
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
    // fImageIndexEdit->setEnabled( false );
    fImageIndexEdit->SetImageIndex(fImageIndex);
}

// Adjust scroll bar of scroll area
void CVolumeViewer::AdjustScrollBar(QScrollBar* nScrollBar, double nFactor)
{
    nScrollBar->setValue(
        int(nFactor * nScrollBar->value() +
            ((nFactor - 1) * nScrollBar->pageStep() / 2)));
}

// cv::Vec2f CVolumeViewer::CleanScrollPosition(cv::Vec2f pos) const
// {
//     int x = pos[0];
//     int y = pos[1];

//     // Get the size of the scroll area viewport
//     int viewportWidth = fScrollArea->viewport()->width();
//     int viewportHeight = fScrollArea->viewport()->height();

//     // Calculate the position of the scroll bars
//     int horizontalPos = x - viewportWidth / 2;
//     int verticalPos = y - viewportHeight / 2;

//     // Check and respect horizontal boundaries
//     if(horizontalPos < fScrollArea->horizontalScrollBar()->minimum())
//         horizontalPos = fScrollArea->horizontalScrollBar()->minimum();
//     else if(horizontalPos > fScrollArea->horizontalScrollBar()->maximum())
//         horizontalPos = fScrollArea->horizontalScrollBar()->maximum();

//     // Check and respect vertical boundaries
//     if(verticalPos < fScrollArea->verticalScrollBar()->minimum())
//         verticalPos = fScrollArea->verticalScrollBar()->minimum();
//     else if(verticalPos > fScrollArea->verticalScrollBar()->maximum())
//         verticalPos = fScrollArea->verticalScrollBar()->maximum();

//     return cv::Vec2f(horizontalPos + viewportWidth / 2, verticalPos + viewportHeight / 2);
// }

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

// void CVolumeViewer::ScrollToCenter(cv::Vec2f pos)
// {    
//     pos = CleanScrollPosition(pos);

//     // Get the size of the scroll area viewport
//     int viewportWidth = fScrollArea->viewport()->width();
//     int viewportHeight = fScrollArea->viewport()->height();

//     // Calculate the position of the scroll bars
//     int horizontalPos = pos[0] - viewportWidth / 2;
//     int verticalPos = pos[1] - viewportHeight / 2;

//     // Set the scroll bar positions
//     fScrollArea->horizontalScrollBar()->setValue(horizontalPos);
//     fScrollArea->verticalScrollBar()->setValue(verticalPos);
// }

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


// cv::Vec2f CVolumeViewer::GetScrollPosition() const
// {
//     // Get the positions of the scroll bars
//     float horizontalPos = static_cast<float>(fScrollArea->horizontalScrollBar()->value() + fScrollArea->viewport()->width() / 2);
//     float verticalPos = static_cast<float>(fScrollArea->verticalScrollBar()->value() + fScrollArea->viewport()->height() / 2);

//     // Return as cv::Vec2f
//     return cv::Vec2f(horizontalPos, verticalPos);
// }

cv::Vec2f CVolumeViewer::GetScrollPosition() const
{
    // Get the positions of the scroll bars
    float horizontalPos = static_cast<float>(fGraphicsView->horizontalScrollBar()->value() + fGraphicsView->viewport()->width() / 2);
    float verticalPos = static_cast<float>(fGraphicsView->verticalScrollBar()->value() + fGraphicsView->viewport()->height() / 2);

    // Return as cv::Vec2f
    return cv::Vec2f(horizontalPos, verticalPos);
}
