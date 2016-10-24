//---------------------------------------------------------------------------------------------------------------------------------------------
// Global_Values.h file for Global_Values Class
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
#include <QMenuBar>
#include <QPixmap>
#include <QRect>
#include "common/types/Rendering.h"
#include "common/vc_defines.h"
#include "volumepkg/volumepkg.h"
#include "volumepkg/volumepkg.h"

class Global_Values
{

public:
    // Determines the status of the thread running texturing
    enum myThreadStatus {
        thread_Inactive,
        thread_Active,
        thread_Successful,
        thread_Cloud_Error,
        thread_Failed,
        thread_Forced_Close
    };

    Global_Values(QRect rec);

    int getHeight();
    int getWidth();

    void createVolumePackage();
    VolumePkg* getVolPkg();

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

    myThreadStatus getStatus();

    void setFileMenu(QMenu* fileMenu);

    void enableMenus(bool value);

    void setInactiveThread();

    void setActiveThread();

    void setThreadSuccessful();

    void setThreadCloudError();

    void setThreadFailed();

    void setThreadForcedClose();

private:
    // Determines the status of the thread running texturing
    // enum myThreadStatus { thread_Inactive, thread_Active, thread_Successful,
    // thread_Cloud_Error, thread_Failed, thread_Forced_Close};

    myThreadStatus status;

    bool VPKG_Instantiated = false;
    int height;
    int width;
    QString path;
    VolumePkg* vpkg;
    std::vector<std::string> segmentations;
    QPixmap pix;
    QMainWindow* _window;
    volcart::Rendering _rendering;
    double _radius;
    int _textureMethod;
    int _sampleDirection;

    QMenu* _fileMenu;
};
