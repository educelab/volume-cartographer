// GlobalValues.cpp file for GlobalValues Class
// Purpose: Implements GlobalValues Class
// Developer: Michael Royal - mgro224@g.uky.edu
// October 12, 2015 - Spring Semester 2016
// Last Updated 10/23/2015 by: Michael Royal

// Copyright 2015 (Brent Seales: Volume Cartography Research)
// University of Kentucky VisCenter

#include "GlobalValues.hpp"

GlobalValues::GlobalValues(QRect rec) : winH_(rec.height()), winW_(rec.width())
{
}

int GlobalValues::getHeight() { return winH_; }

int GlobalValues::getWidth() { return winW_; }

volcart::VolumePkg::Pointer GlobalValues::volPkg() { return vpkg_; }

void GlobalValues::setVolgPkgPath(QString p) { vpkgPath_ = std::move(p); }

void GlobalValues::loadVolPkg()
{
    vpkg_ = volcart::VolumePkg::New(vpkgPath_.toStdString());
}

void GlobalValues::unloadVolPkg()
{
    vpkg_ = nullptr;
    activeSeg_ = nullptr;
}

bool GlobalValues::pkgLoaded() { return vpkg_ != nullptr; }

void GlobalValues::resetGUI()
{
    vpkgPath_.clear();
    vpkg_ = nullptr;
    activeSeg_ = nullptr;
    segIDs_.clear();
    clearRendering();
    radius_ = 0;
    textureMethod_ = Method::Intersection;
    sampleDirection_ = 0;
    status_ = ThreadStatus::Inactive;
}

void GlobalValues::loadSegIDs() { segIDs_ = vpkg_->segmentationIDs(); }

std::vector<std::string> GlobalValues::getSegIDs() { return segIDs_; }

void GlobalValues::setQPixMapImage(QImage image)
{
    pix_ = QPixmap::fromImage(image);
}

QPixmap GlobalValues::getQPixMapImage() { return pix_; }

void GlobalValues::setWindow(QMainWindow* window) { window_ = window; }

QMainWindow* GlobalValues::getWindow() { return window_; }

void GlobalValues::setRendering(Rendering r) { rendering_ = std::move(r); }

void GlobalValues::clearRendering() { rendering_ = Rendering(); }

Rendering GlobalValues::getRendering() { return rendering_; }

void GlobalValues::setRadius(double radius) { radius_ = radius; }

double GlobalValues::getRadius() { return radius_; }

void GlobalValues::setTextureMethod(GlobalValues::Method textureMethod)
{
    textureMethod_ = textureMethod;
}

GlobalValues::Method GlobalValues::getTextureMethod() { return textureMethod_; }

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
    sampleDirection_ = sampleDirection;
}

int GlobalValues::getSampleDirection() { return sampleDirection_; }

void GlobalValues::setThreadStatus(ThreadStatus status) { status_ = status; };

ThreadStatus GlobalValues::getStatus() { return status_; }

void GlobalValues::setFileMenu(QMenu* fileMenu) { fileMenu_ = fileMenu; }

void GlobalValues::enableMenus(bool value)
{
    int numElements = fileMenu_->actions().size();
    for (int i = 0; i < numElements; i++) {
        fileMenu_->actions().at(i)->setEnabled(value);
    }
}
