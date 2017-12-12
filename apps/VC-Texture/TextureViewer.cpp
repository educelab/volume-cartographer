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

#include "TextureViewer.hpp"
#include <qmainwindow.h>

TextureViewer::TextureViewer(GlobalValues* globals) : globals_{globals}
{
    // LEFT SIDE OF GUI
    //*****************

    imageLabel_ = new QLabel;
    imageLabel_->setBackgroundRole(QPalette::Base);
    imageLabel_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    imageLabel_->setScaledContents(true);

    scrollArea_ = new QScrollArea;
    scrollArea_->setBackgroundRole(QPalette::Dark);
    scrollArea_->show();

    // Create Buttons
    zoomIn_ = new QPushButton("Zoom In");
    zoomOut_ = new QPushButton("Zoom Out");
    refresh_ = new QPushButton("Reset");
    spacer_ = new QLabel();

    zoomIn_->setMaximumSize(150, 50);
    zoomOut_->setMaximumSize(150, 50);
    refresh_->setMaximumSize(150, 50);
    zoomIn_->setMinimumSize(150, 50);
    zoomOut_->setMinimumSize(150, 50);
    refresh_->setMinimumSize(150, 50);

    progressBar_ = new QProgressBar();
    progressBar_->setVisible(false);
    progressBar_->setMinimumWidth(100);
    progressBar_->setMinimumHeight(10);
    progressBar_->setMinimum(0);
    progressBar_->setMaximum(0);

    // Default Not Enabled
    zoomIn_->setEnabled(false);
    zoomOut_->setEnabled(false);
    refresh_->setEnabled(false);

    viewer_ = new QLabel("Viewer");
    viewer_->setMaximumSize(50, 30);

    zoomPanel_ = new QHBoxLayout();
    zoomPanel_->addWidget(spacer_);
    zoomPanel_->addWidget(progressBar_);
    zoomPanel_->addWidget(zoomIn_);
    zoomPanel_->addWidget(zoomOut_);
    zoomPanel_->addWidget(refresh_);
    viewerPanel_ = new QVBoxLayout();
    viewerPanel_->addWidget(viewer_);
    viewerPanel_->addWidget(scrollArea_);
    viewerPanel_->addLayout(zoomPanel_);

    // END OF LEFT SIDE OF SCREEN
    //***************************

    setup_actions_();
}

// PRIVATE SLOTS //
void TextureViewer::zoom_in_() { scale_image_(1.25); }

void TextureViewer::zoom_out_() { scale_image_(0.8); }

void TextureViewer::zoom_reset_()
{
    imageLabel_->adjustSize();
    scaleFactor_ = 1.0;
    zoomIn_->setEnabled(true);
    zoomOut_->setEnabled(true);
}

// END OF PRIVATE SLOTS

QVBoxLayout* TextureViewer::getLayout() { return viewerPanel_; }

void TextureViewer::loadImageFromGlobals()
{
    imageLabel_ = new QLabel;
    imageLabel_->setBackgroundRole(QPalette::Base);
    imageLabel_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    imageLabel_->setScaledContents(true);

    pix_ = globals_->getQPixMapImage();
    imageLabel_->setPixmap(pix_);
    scrollArea_->setWidget(imageLabel_);

    if (!imageLabel_->pixmap()->isNull()) {
        zoomIn_->setEnabled(true);
        zoomOut_->setEnabled(true);
        refresh_->setEnabled(true);
    }

    imageLabel_->adjustSize();
    scaleFactor_ = 1.0;
}

void TextureViewer::clearImageArea()
{
    imageLabel_->adjustSize();
    scaleFactor_ = 1.0;
    imageLabel_->close();
    zoomIn_->setEnabled(false);
    zoomOut_->setEnabled(false);
    refresh_->setEnabled(false);
}

void TextureViewer::clearGUI()
{

    clearImageArea();
    scaleFactor_ = 1.0;
}

void TextureViewer::setup_actions_()
{
    zoomInAction_ = new QAction("Zoom In", this);
    zoomInAction_->setEnabled(false);
    connect(zoomIn_, SIGNAL(released()), this, SLOT(zoom_in_()));

    zoomOutAction_ = new QAction("Zoom Out", this);
    zoomOutAction_->setEnabled(false);
    connect(zoomOut_, SIGNAL(released()), this, SLOT(zoom_out_()));

    zoomResetAction_ = new QAction("Reset", this);
    zoomResetAction_->setEnabled(false);
    connect(refresh_, SIGNAL(released()), this, SLOT(zoom_reset_()));
}

void TextureViewer::adjust_scroll_bar_(QScrollBar* scrollBar, double factor)
{
    scrollBar->setValue(
        int(factor * scrollBar->value() +
            ((factor - 1) * scrollBar->pageStep() / 2)));
}

void TextureViewer::scale_image_(double factor)
{
    if (scaleFactor_ * factor <= 2.0 && scaleFactor_ * factor >= 0.15) {

        scaleFactor_ *= factor;
        zoomIn_->setEnabled(scaleFactor_ * 1.25 < 2.0);
        zoomOut_->setEnabled(scaleFactor_ * 0.8 > 0.15);

        imageLabel_->resize(scaleFactor_ * imageLabel_->pixmap()->size());

        adjust_scroll_bar_(scrollArea_->horizontalScrollBar(), factor);
        adjust_scroll_bar_(scrollArea_->verticalScrollBar(), factor);
    }
}

void TextureViewer::setProgressActive(bool value)
{
    progressBar_->setVisible(value);
}

void TextureViewer::setEnabled(bool value)
{
    zoomIn_->setEnabled(value);
    zoomOut_->setEnabled(value);
    refresh_->setEnabled(value);
}

void TextureViewer::clearLabel()  // Clears Label
{
    imageLabel_->adjustSize();
    scaleFactor_ = 1.0;
    imageLabel_->close();
}
