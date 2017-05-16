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
#include "vc/core/types/Rendering.hpp"
#include "vc/core/types/VolumePkg.hpp"

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
    GlobalValues(QRect rec);

    int getHeight();
    int getWidth();

    void createVolumePackage();
    volcart::VolumePkg* getVolPkg();

    void clearVolumePackage();

    void clearGUI();

    void setPath(QString newPath);

    void getMySegmentations();
    std::vector<std::string> getSegmentations();

    void setQPixMapImage(QImage image);
    QPixmap getQPixMapImage();

    bool isVPKG_Intantiated();

    void setWindow(QMainWindow* window);
    QMainWindow* getWindow();

    void setRendering(volcart::Rendering rendering);
    void clearRendering();
    volcart::Rendering getRendering();

    void setRadius(double radius);
    double getRadius();

    void setTextureMethod(int textureMethod);
    int getTextureMethod();

    void setSampleDirection(int sampleDirection);
    int getSampleDirection();

    ThreadStatus getStatus();

    void setFileMenu(QMenu* fileMenu);

    void enableMenus(bool value);

    void setThreadStatus(ThreadStatus status);

private:
    // The status of the thread running the texturing process
    ThreadStatus _status;

    bool VPKG_Instantiated = false;
    int height;
    int width;
    QString path;
    volcart::VolumePkg* vpkg;
    std::vector<std::string> segmentations;
    QPixmap pix;
    QMainWindow* _window;
    volcart::Rendering _rendering;
    double _radius;
    int _textureMethod;
    int _sampleDirection;

    QMenu* _fileMenu;
};
