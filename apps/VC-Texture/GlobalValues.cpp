//----------------------------------------------------------------------------------------------------------------------------------------
// GlobalValues.cpp file for GlobalValues Class
// Purpose: Implements GlobalValues Class
// Developer: Michael Royal - mgro224@g.uky.edu
// October 12, 2015 - Spring Semester 2016
// Last Updated 10/23/2015 by: Michael Royal

// Copyright 2015 (Brent Seales: Volume Cartography Research)
// University of Kentucky VisCenter
//----------------------------------------------------------------------------------------------------------------------------------------

#include "GlobalValues.hpp"

GlobalValues::GlobalValues(QRect rec)
    : _status(ThreadStatus::Inactive)
    , height(rec.height())
    , width(rec.width())
    , _radius(0)
    , _textureMethod(Method::Intersection)
    , _sampleDirection(0)
{
}

int GlobalValues::getHeight() { return height; }

int GlobalValues::getWidth() { return width; }

volcart::VolumePkg::Pointer GlobalValues::getVolPkg() { return vpkg; }

void GlobalValues::setPath(QString newPath) { path = newPath; }

void GlobalValues::createVolumePackage()
{
    // Creates a Volume Package
    vpkg = volcart::VolumePkg::New(path.toStdString());
    VpkgInstantiated = true;
}

void GlobalValues::clearVolumePackage()
{
    vpkg = nullptr;
    activeSeg = nullptr;
    VpkgInstantiated = false;
}

void GlobalValues::clearGUI()
{
    VpkgInstantiated = false;
    path.clear();
    vpkg = nullptr;
    activeSeg = nullptr;
    segmentations.clear();
    clearRendering();
    _radius = 0;
    _textureMethod = Method::Intersection;
    _sampleDirection = 0;
    _status = ThreadStatus::Inactive;
}

void GlobalValues::getMySegmentations()
{
    segmentations = vpkg->segmentationIDs();
}

std::vector<std::string> GlobalValues::getSegmentations()
{
    return segmentations;
}

void GlobalValues::setQPixMapImage(QImage image)
{
    pix = QPixmap::fromImage(image);
}

QPixmap GlobalValues::getQPixMapImage() { return pix; }

bool GlobalValues::isVpkgInstantiated() { return VpkgInstantiated; }

void GlobalValues::setWindow(QMainWindow* window) { _window = window; }

QMainWindow* GlobalValues::getWindow() { return _window; }

void GlobalValues::setRendering(Rendering rendering)
{
    _rendering = std::move(rendering);
}

void GlobalValues::clearRendering() { _rendering = Rendering(); }

Rendering GlobalValues::getRendering() { return _rendering; }

void GlobalValues::setRadius(double radius) { _radius = radius; }

double GlobalValues::getRadius() { return _radius; }

void GlobalValues::setTextureMethod(GlobalValues::Method textureMethod)
{
    _textureMethod = textureMethod;
}

GlobalValues::Method GlobalValues::getTextureMethod() { return _textureMethod; }

void GlobalValues::setFilter(volcart::texturing::CompositeTexture::Filter f)
{
    textureFilter_ = f;
}

volcart::texturing::CompositeTexture::Filter GlobalValues::getFilter()
{
    return textureFilter_;
}

void GlobalValues::setSampleDirection(int sampleDirection)
{
    _sampleDirection = sampleDirection;
}

int GlobalValues::getSampleDirection() { return _sampleDirection; }

void GlobalValues::setThreadStatus(ThreadStatus status) { _status = status; };

ThreadStatus GlobalValues::getStatus() { return _status; }

void GlobalValues::setFileMenu(QMenu* fileMenu) { _fileMenu = fileMenu; }

void GlobalValues::enableMenus(bool value)
{
    int numElements = _fileMenu->actions().size();
    for (int i = 0; i < numElements; i++) {
        _fileMenu->actions().at(i)->setEnabled(value);
    }
}
