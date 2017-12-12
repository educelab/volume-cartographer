// GlobalValues.h file for GlobalValues Class
// Purpose: Used to pass values through the Program, Data that needs to be
// shared between several Objects should be declared in globals
// Developer: Michael Royal - mgro224@g.uky.edu
// October 12, 2015 - Spring Semester 2016
// Last Updated 10/23/2015 by: Michael Royal

// Copyright 2015 (Brent Seales: Volume Cartography Research)
// University of Kentucky VisCenter

#pragma once

#include <QApplication>
#include <QImage>
#include <QMainWindow>
#include <QMenu>
#include <QPixmap>
#include <QRect>

#include "Rendering.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/texturing/CompositeTexture.hpp"

// Determines the status of the thread running texturing
enum ThreadStatus {
    Inactive,
    Active,
    Successful,
    CloudError,
    Failed,
    ForcedClose
};

class GlobalValues
{

public:
    enum Method { Intersection = 0, Integral, Composite };

    explicit GlobalValues(QRect rec);

    int getHeight();
    int getWidth();

    void setVolgPkgPath(QString p);
    void loadVolPkg();
    void unloadVolPkg();
    bool pkgLoaded();
    volcart::VolumePkg::Pointer volPkg();

    void setActiveSeg(const std::string& id)
    {
        activeSeg_ = vpkg_->segmentation(id);
    }
    volcart::Segmentation::Pointer getActiveSegmentation()
    {
        return activeSeg_;
    }

    void resetGUI();

    void loadSegIDs();
    std::vector<std::string> getSegIDs();

    void setQPixMapImage(QImage image);
    QPixmap getQPixMapImage();

    void setWindow(QMainWindow* window);
    QMainWindow* getWindow();

    void setRendering(Rendering r);
    void clearRendering();
    Rendering getRendering();

    void setRadius(double radius);
    double getRadius();

    void setTextureMethod(Method textureMethod);
    Method getTextureMethod();

    void setFilter(volcart::texturing::CompositeTexture::Filter f);
    volcart::texturing::CompositeTexture::Filter getFilter();

    void setSampleDirection(int sampleDirection);
    int getSampleDirection();

    ThreadStatus getStatus();

    void setFileMenu(QMenu* fileMenu);

    void enableMenus(bool value);

    void setThreadStatus(ThreadStatus status);

private:
    // The status of the thread running the texturing process
    ThreadStatus status_{ThreadStatus::Inactive};

    int winH_;
    int winW_;
    QString vpkgPath_;
    volcart::VolumePkg::Pointer vpkg_;
    volcart::Segmentation::Pointer activeSeg_;
    std::vector<std::string> segIDs_;
    QPixmap pix_;
    QMainWindow* window_;
    Rendering rendering_;
    double radius_{1};
    Method textureMethod_{Method::Intersection};
    volcart::texturing::CompositeTexture::Filter textureFilter_{
        volcart::texturing::CompositeTexture::Filter::Minimum};
    int sampleDirection_{0};

    QMenu* fileMenu_;
};
