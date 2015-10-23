//-----------------------------------------------------------------------------------------------------------------------------------------
// Texture_Viewer.cpp file for Texture_Viewer Class
// Purpose: Define Texture_Viewer Class
// Developer: Michael Royal - mgro224@g.uky.edu
// October 12, 2015 - Spring Semester 2016
// Last Updated 10/23/2015 by: Michael Royal
// Code Reference: http://doc.qt.io/qt-5/qtwidgets-widgets-imageviewer-example.html ---(I edited/formatted the code to suit our purposes)
//-----------------------------------------------------------------------------------------------------------------------------------------

#include <qmainwindow.h>
#include "Texture_Viewer.h"

Texture_Viewer::Texture_Viewer(DefaultValues *values)
{
    defaultVals = values;

    //LEFT SIDE OF GUI
    //*****************************************************************************************

    scaleFactor=1;// Instantiates scaleFactor to 1 for Resizing

    QPixmap pix;
    pix.load("/Users/Royal/Desktop/Volume_Car/volume-cartographer/apps/textureUI/Space.jpg");

    imageLabel = new QLabel;
    imageLabel->setBackgroundRole(QPalette::Base);
    imageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    imageLabel->setScaledContents(true);
    imageLabel->setPixmap(pix);

    scrollArea = new QScrollArea;
    scrollArea->setBackgroundRole(QPalette::Dark);
    scrollArea->setWidget(imageLabel);
    scrollArea->show();

    // Create Buttons
    zoomIn = new QPushButton("Zoom In");// Creates a QPushButton - Zoom_In
    zoomOut = new QPushButton("Zoom Out");// Creates a QPushButton - Zoom_Out
    refresh = new QPushButton ("Reset");// Create a QPushButton - Refresh
    spacer = new QLabel();
    zoomIn->setMaximumSize(150,50);// Sets Max Size for Zoom_In Button
    zoomOut->setMaximumSize(150,50);// Sets Max Size for Zoom_Out Button
    refresh->setMaximumSize(150,50);// Sets Max Size for Refresh Button

    viewer = new QLabel("Viewer");
    viewer->setMaximumSize(50,30);

    zoom = new QHBoxLayout();// Lines Up the Following Buttons Horizontally
    zoom->addWidget(spacer);
    zoom->addWidget(zoomIn);// Zoom_In Button added to Horizontal_Layout
    zoom->addWidget(zoomOut);// Zoom_Out Button added to Horizontal_Layout
    zoom->addWidget(refresh);// Refresh Button added to Horizontal_Layout

    image_Management = new QVBoxLayout();// Manages Left segment of screen - QWidgets
    image_Management->addWidget(viewer);
    image_Management->addWidget(scrollArea);
    image_Management->addLayout(zoom);

    //END OF LEFT SIDE OF SCREEN
    //********************************************************************************************

    create_Actions();

}// End of Texture_Viewer() default Constructor

// PRIVATE SLOTS
//--------------------------------------------

void Texture_Viewer::open()
{
   // Implement Code Here

}// End of Texture_Viewer::Open()


void Texture_Viewer::zoom_In()
{
    scale_Texture(1.25);

}// End of Texture_Viewer::zoom_In()


void Texture_Viewer::zoom_Out()
{
    scale_Texture(0.8);

}// End of Texture_Viewer::zoom_Out()


void Texture_Viewer::reset_Size()
{
    zoomIn->setEnabled(true);
    zoomOut->setEnabled(true);
    imageLabel->adjustSize();
    scaleFactor = 1.0;

}// End of Texture_Viewer::reset_Size()


// END OF PRIVATE SLOTS
//----------------------------------------------

QVBoxLayout * Texture_Viewer::getLayout()
{
    return image_Management;

}// End of Texture_Viewer::getLayout()


void Texture_Viewer::create_Actions()
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

}// End of Texture_Viewer::create_Actions()

void Texture_Viewer::adjustScrollBar(QScrollBar *scrollBar, double factor)
{
    scrollBar->setValue(int(factor * scrollBar->value() + ((factor - 1) * scrollBar->pageStep()/2)));

}// End of Texture_Viewer::adjustScrollBar(QScrollBar *scrollBar, double factor)


void Texture_Viewer::scale_Texture(double factor)
{
    if(scaleFactor*factor <= 2.0 && scaleFactor*factor >=0.15)
    {
        scaleFactor *= factor;

        if(scaleFactor*1.25>2.0)
        {
            zoomIn->setEnabled(false);

        }else zoomIn->setEnabled(true);

        if(scaleFactor*0.8<0.15)
        {
            zoomOut->setEnabled(false);

        }else zoomOut->setEnabled(true);

        imageLabel->resize(scaleFactor * imageLabel->pixmap()->size());

        adjustScrollBar(scrollArea->horizontalScrollBar(), factor);
        adjustScrollBar(scrollArea->verticalScrollBar(), factor);

    }

}// End of Texture_Viewer::scale_Texture(double factor)









