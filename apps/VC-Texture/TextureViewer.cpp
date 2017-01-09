//-----------------------------------------------------------------------------------------------------------------------------------------
// TextureViewer.cpp file for TextureViewer Class
// Purpose: Define TextureViewer Class
// Developer: Michael Royal - mgro224@g.uky.edu
// October 12, 2015 - Spring Semester 2016
// Last Updated 10/23/2015 by: Michael Royal
// Code Reference:
// http://doc.qt.io/qt-5/qtwidgets-widgets-imageviewer-example.html ---(I
// edited/formatted the code to suit our purposes)

// Copyright 2015 (Brent Seales: Volume Cartography Research)
// University of Kentucky VisCenter
//-----------------------------------------------------------------------------------------------------------------------------------------

#include "TextureViewer.h"
#include <qmainwindow.h>

TextureViewer::TextureViewer(GlobalValues* globals)
{
    _globals = globals;

    // LEFT SIDE OF GUI
    //*****************************************************************************************

    scaleFactor = 1;  // Instantiates scaleFactor to 1 for Resizing

    imageLabel = new QLabel;
    imageLabel->setBackgroundRole(QPalette::Base);
    imageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    imageLabel->setScaledContents(true);

    scrollArea = new QScrollArea;
    scrollArea->setBackgroundRole(QPalette::Dark);
    scrollArea->show();

    // Create Buttons
    //------------------------------------------------------------------------
    zoomIn = new QPushButton("Zoom In");    // Creates a QPushButton - Zoom_In
    zoomOut = new QPushButton("Zoom Out");  // Creates a QPushButton - Zoom_Out
    refresh = new QPushButton("Reset");     // Create a QPushButton - Refresh
    spacer = new QLabel();

    zoomIn->setMaximumSize(150, 50);   // Sets Max Size for Zoom_In Button
    zoomOut->setMaximumSize(150, 50);  // Sets Max Size for Zoom_Out Button
    refresh->setMaximumSize(150, 50);  // Sets Max Size for Refresh Button
    zoomIn->setMinimumSize(150, 50);   // Sets Min Size for Zoom_In Button
    zoomOut->setMinimumSize(150, 50);  // Sets Min Size for Zoom_Out Button
    refresh->setMinimumSize(150, 50);  // Sets Min Size for Refresh Button

    cancel = new QPushButton("Cancel");
    cancel->setMaximumSize(150, 50);  // Sets Max Size for Cancel Button
    cancel->setMinimumSize(150, 50);  // Sets Min Size for Cancel Button
    cancel->setVisible(false);
    //----------------------------------------------------------------------

    progressBar = new QProgressBar();
    progressBar->setVisible(false);
    progressBar->setMinimumWidth(100);
    progressBar->setMinimumHeight(10);
    progressBar->setMinimum(0);
    progressBar->setMaximum(0);

    // Default Not Enabled
    zoomIn->setEnabled(false);
    zoomOut->setEnabled(false);
    refresh->setEnabled(false);
    cancel->setEnabled(false);

    viewer = new QLabel("Viewer");
    viewer->setMaximumSize(50, 30);

    zoom = new QHBoxLayout();  // Lines Up the Following Buttons Horizontally
    zoom->addWidget(spacer);
    zoom->addWidget(progressBar);
    zoom->addWidget(cancel);
    zoom->addWidget(zoomIn);   // Zoom_In Button added to Horizontal_Layout
    zoom->addWidget(zoomOut);  // Zoom_Out Button added to Horizontal_Layout
    zoom->addWidget(refresh);  // Refresh Button added to Horizontal_Layout

    image_Management =
        new QVBoxLayout();  // Manages Left segment of screen - QWidgets
    image_Management->addWidget(viewer);
    image_Management->addWidget(scrollArea);
    image_Management->addLayout(zoom);

    // END OF LEFT SIDE OF SCREEN
    //********************************************************************************************

    create_Actions();

}  // End of TextureViewer() default Constructor

// PRIVATE SLOTS
//--------------------------------------------

void TextureViewer::open()
{
    // Implement Code Here

}  // End of TextureViewer::Open()

void TextureViewer::zoom_In()
{
    scale_Texture(1.25);

}  // End of TextureViewer::zoom_In()

void TextureViewer::zoom_Out()
{
    scale_Texture(0.8);

}  // End of TextureViewer::zoom_Out()

void TextureViewer::reset_Size()
{
    imageLabel->adjustSize();
    scaleFactor = 1.0;
    zoomIn->setEnabled(true);
    zoomOut->setEnabled(true);

}  // End of TextureViewer::reset_Size()

void TextureViewer::quitThread()
{
    _globals->setThreadStatus(ThreadStatus::ForcedClose);
}

// END OF PRIVATE SLOTS
//----------------------------------------------

QVBoxLayout* TextureViewer::getLayout()
{
    return image_Management;

}  // End of TextureViewer::getLayout()

void TextureViewer::setImage()  // Minor Glitch, delay zooms, ext. Still needs
                                // some Fixing
{
    imageLabel = new QLabel;
    imageLabel->setBackgroundRole(QPalette::Base);
    imageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    imageLabel->setScaledContents(true);

    pix = _globals->getQPixMapImage();
    imageLabel->setPixmap(pix);
    scrollArea->setWidget(imageLabel);

    if (!imageLabel->pixmap()->isNull()) {
        zoomIn->setEnabled(true);
        zoomOut->setEnabled(true);
        refresh->setEnabled(true);
    }

    imageLabel->adjustSize();
    scaleFactor = 1.0;
}

void TextureViewer::clearImageLabel()
{
    imageLabel->adjustSize();
    scaleFactor = 1.0;
    imageLabel->close();
    zoomIn->setEnabled(false);
    zoomOut->setEnabled(false);
    refresh->setEnabled(false);
}

void TextureViewer::clearGUI()
{

    clearImageLabel();  // Clear the Image
    scaleFactor = 1.0;
}

void TextureViewer::create_Actions()
{
    zoomInAction = new QAction("Zoom In", this);
    zoomInAction->setEnabled(false);
    connect(zoomIn, SIGNAL(released()), this, SLOT(zoom_In()));

    zoomOutAction = new QAction("Zoom Out", this);
    zoomOutAction->setEnabled(false);
    connect(zoomOut, SIGNAL(released()), this, SLOT(zoom_Out()));

    resetSizeAction = new QAction("Reset", this);
    resetSizeAction->setEnabled(false);
    connect(refresh, SIGNAL(released()), this, SLOT(reset_Size()));

    _cancel = new QAction("Cancel", this);
    _cancel->setEnabled(true);
    connect(cancel, SIGNAL(released()), this, SLOT(quitThread()));

}  // End of TextureViewer::create_Actions()

void TextureViewer::adjustScrollBar(QScrollBar* scrollBar, double factor)
{
    scrollBar->setValue(
        int(factor * scrollBar->value() +
            ((factor - 1) * scrollBar->pageStep() / 2)));

}  // End of TextureViewer::adjustScrollBar(QScrollBar *scrollBar, double
   // factor)

void TextureViewer::scale_Texture(double factor)
{
    if (scaleFactor * factor <= 2.0 && scaleFactor * factor >= 0.15) {
        scaleFactor *= factor;

        if (scaleFactor * 1.25 > 2.0) {
            zoomIn->setEnabled(false);

        } else
            zoomIn->setEnabled(true);

        if (scaleFactor * 0.8 < 0.15) {
            zoomOut->setEnabled(false);

        } else
            zoomOut->setEnabled(true);

        imageLabel->resize(scaleFactor * imageLabel->pixmap()->size());

        adjustScrollBar(scrollArea->horizontalScrollBar(), factor);
        adjustScrollBar(scrollArea->verticalScrollBar(), factor);
    }

}  // End of TextureViewer::scale_Texture(double factor)

void TextureViewer::progressActive(bool value)
{
    progressBar->setVisible(value);
    // cancel->setVisible(value);
    // cancel->setEnabled(value);
}

void TextureViewer::setEnabled(bool value)
{
    zoomIn->setEnabled(value);
    zoomOut->setEnabled(value);
    refresh->setEnabled(value);
}

void TextureViewer::clearLabel()  // Clears Label
{
    imageLabel->adjustSize();
    scaleFactor = 1.0;
    imageLabel->close();
}
