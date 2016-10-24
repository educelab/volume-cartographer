//----------------------------------------------------------------------------------------------------------------------------------------
// Global_Values.cpp file for Global_Values Class
// Purpose: Implements Global_Values Class
// Developer: Michael Royal - mgro224@g.uky.edu
// October 12, 2015 - Spring Semester 2016
// Last Updated 10/23/2015 by: Michael Royal

// Copyright 2015 (Brent Seales: Volume Cartography Research)
// University of Kentucky VisCenter
//----------------------------------------------------------------------------------------------------------------------------------------

#include "Global_Values.h"

Global_Values::Global_Values(QRect rec)

    : height(rec.height())
    , width(rec.width())
    , VPKG_Instantiated(false)
    , _radius(0)
    , _textureMethod(0)
    , _sampleDirection(0)
    , status(thread_Inactive)

{ /* Do Nothing*/
}  // End of Default Constructor()

int Global_Values::getHeight() { return height; }

int Global_Values::getWidth() { return width; }

VolumePkg* Global_Values::getVolPkg() { return vpkg; }

void Global_Values::setPath(QString newPath) { path = newPath; }

void Global_Values::createVolumePackage()
{
    vpkg = new VolumePkg(path.toStdString());  // Creates a Volume Package
    VPKG_Instantiated = true;
}

void Global_Values::clearVolumePackage()
{
    vpkg = nullptr;             // Clear vpkg
    VPKG_Instantiated = false;  // Clear VPKG_Instantiated
}

void Global_Values::clearGUI()
{
    VPKG_Instantiated = false;
    path = "";
    vpkg = nullptr;
    segmentations.clear();
    clearRendering();
    _radius = 0;
    _textureMethod = 0;
    _sampleDirection = 0;
    status = thread_Inactive;
}

void Global_Values::getMySegmentations()
{
    segmentations = vpkg->getSegmentations();
}

std::vector<std::string> Global_Values::getSegmentations()
{
    return segmentations;
}

void Global_Values::setQPixMapImage(QImage image)
{
    pix = QPixmap::fromImage(image);
}

QPixmap Global_Values::getQPixMapImage() { return pix; }

bool Global_Values::isVPKG_Intantiated() { return VPKG_Instantiated; }

void Global_Values::setWindow(QMainWindow* window) { _window = window; }

QMainWindow* Global_Values::getWindow() { return _window; }

void Global_Values::setRendering(volcart::Rendering rendering)
{
    _rendering = rendering;
}

void Global_Values::clearRendering()
{
    volcart::Rendering* empty = new volcart::Rendering;
    _rendering = *empty;
}

volcart::Rendering Global_Values::getRendering() { return _rendering; }

void Global_Values::setRadius(double radius) { _radius = radius; }

double Global_Values::getRadius() { return _radius; }

void Global_Values::setTextureMethod(int textureMethod)
{
    _textureMethod = textureMethod;
}

int Global_Values::getTextureMethod() { return _textureMethod; }

void Global_Values::setSampleDirection(int sampleDirection)
{
    _sampleDirection = sampleDirection;
}

int Global_Values::getSampleDirection() { return _sampleDirection; }

// void Global_Values::setStatus(myThreadStatus myStatus) { status = myStatus; }

void Global_Values::setInactiveThread()
{
    status = myThreadStatus::thread_Inactive;
};

void Global_Values::setActiveThread()
{
    status = myThreadStatus::thread_Active;
};

void Global_Values::setThreadSuccessful()
{
    status = myThreadStatus::thread_Successful;
};

void Global_Values::setThreadCloudError()
{
    status = myThreadStatus::thread_Cloud_Error;
};

void Global_Values::setThreadFailed()
{
    status = myThreadStatus::thread_Failed;
};

void Global_Values::setThreadForcedClose()
{
    status = myThreadStatus::thread_Forced_Close;
};

Global_Values::myThreadStatus Global_Values::getStatus() { return status; }

void Global_Values::setFileMenu(QMenu* fileMenu) { _fileMenu = fileMenu; }

void Global_Values::enableMenus(bool value) { _fileMenu->setEnabled(value); }
