//---------------------------------------------------------------------------------------------------------------------------------------------
// GlobalValues.h file for GlobalValues Class
// Purpose: Used to pass values through the Program, Data that needs to be
// shared between several Objects should be declared in globals
// Developer: Michael Royal - mgro224@g.uky.edu
// October 12, 2015 - Spring Semester 2016
// Last Updated 10/23/2015 by: Michael Royal

// Copyright 2015 (Brent Seales: Volume Cartography Research)
// University of Kentucky VisCenter
//---------------------------------------------------------------------------------------------------------------------------------------------

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

// Determines the _status of the thread running texturing
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

    void createVolumePackage();
    volcart::VolumePkg::Pointer getVolPkg();

    void setActiveSegmentation(const std::string& id)
    {
        activeSeg = vpkg->segmentation(id);
    }
    volcart::Segmentation::Pointer getActiveSegmentation() { return activeSeg; }

    void clearVolumePackage();

    void clearGUI();

    void setPath(QString newPath);

    void getMySegmentations();
    std::vector<std::string> getSegmentations();

    void setQPixMapImage(QImage image);
    QPixmap getQPixMapImage();

    bool isVpkgInstantiated();

    void setWindow(QMainWindow* window);
    QMainWindow* getWindow();

    void setRendering(Rendering rendering);
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
    ThreadStatus _status;

    bool VpkgInstantiated{false};
    int height;
    int width;
    QString path;
    volcart::VolumePkg::Pointer vpkg;
    volcart::Segmentation::Pointer activeSeg;
    std::vector<std::string> segmentations;
    QPixmap pix;
    QMainWindow* _window;
    Rendering _rendering;
    double _radius;
    Method _textureMethod;
    volcart::texturing::CompositeTexture::Filter textureFilter_;
    int _sampleDirection;

    QMenu* _fileMenu;
};
