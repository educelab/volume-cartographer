//----------------------------------------------------------------------------------------------------------------------------------------
// Segmentations_Viewer.h file for Segmentations_Viewer Class
// Purpose: Create header file for Segmentations_Viewer Class
// Developer: Michael Royal - mgro224@g.uky.edu
// October 12, 2015 - Spring Semester 2016
// Last Updated 09/26/2016 by: Michael Royal

// Copy Right ©2015 (Brent Seales: Volume Cartography Research) - University of Kentucky Center for Visualization and Virtualization
//----------------------------------------------------------------------------------------------------------------------------------------

#pragma once

#include "common/io/ply2itk.h"
#include "texturing/compositeTexture.h"
#include "common/vc_defines.h"
#include "volumepkg/volumepkg.h"

#include "Global_Values.h"
#include "Texture_Viewer.h"
#include "MyThread.h"

#include <QObject>
#include <QLabel>
#include <QListWidget>
#include <QVBoxLayout>
#include <QSpinBox>
#include <QComboBox>
#include <QPushButton>
#include <QFormLayout>
#include <QSize>
#include <QMessageBox>

class Segmentations_Viewer:QObject
{
    // NOTICE THIS MACRO
Q_OBJECT
    //

public:
    Segmentations_Viewer(Global_Values *globals, Texture_Viewer *texture_Viewer);
    QVBoxLayout * getLayout();
    void setSegmentations();
    bool loadImage(cv::Mat texture);
    void setVol_Package_Name(QString name);

    public slots:
        void itemClickedSlot();
        void generateTextureImage();
        void setEnabled(bool value);

private:

    MyThread *processing;

    Texture_Viewer *_texture_Viewer;

    QVBoxLayout *panels;// Main Layout for Right Side of GUI

    QLabel *volume_Package;
    QListWidget *segmentations;
    QString currentSegmentation;

    QLabel *parameters;
    Global_Values *_globals;
    QSpinBox *radius;
    QComboBox *texture_Method;
    QComboBox *sample_Direction;
    QPushButton *generate;

    QFormLayout *inputs;
    QVBoxLayout *user_input;

    QImage newImage;

};
