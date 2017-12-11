//----------------------------------------------------------------------------------------------------------------------------------------
// SegmentationsViewer.h file for SegmentationsViewer Class
// Purpose: Create header file for SegmentationsViewer Class
// Developer: Michael Royal - mgro224@g.uky.edu
// October 12, 2015 - Spring Semester 2016
// Last Updated 09/26/2016 by: Michael Royal

// Copyright 2015 (Brent Seales: Volume Cartography Research)
// University of Kentucky VisCenter
//----------------------------------------------------------------------------------------------------------------------------------------

#pragma once

#include "vc/core/io/PLYReader.hpp"
#include "vc/core/types/VolumePkg.hpp"

#include "GlobalValues.hpp"
#include "MyThread.hpp"
#include "TextureViewer.hpp"

#include <QComboBox>
#include <QFormLayout>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QObject>
#include <QPushButton>
#include <QSize>
#include <QSpinBox>
#include <QVBoxLayout>

class SegmentationsViewer : QObject
{
    // NOTICE THIS MACRO
    Q_OBJECT
    //

public:
    SegmentationsViewer(GlobalValues* globals, TextureViewer* texture_Viewer);
    QVBoxLayout* getLayout();
    void clearGUI();
    void setSegmentations();
    bool loadImage(cv::Mat texture);
    void setVol_Package_Name(QString name);

public slots:
    void itemClickedSlot();
    void generateTextureImage();
    void setEnabled(bool value);

private:
    MyThread* processing;
    GlobalValues* _globals;
    TextureViewer* _texture_Viewer;

    QVBoxLayout* panels;
    QLabel* volume_Package;
    QListWidget* segmentations;
    QString currentSegmentation;
    QLabel* parameters;
    QSpinBox* radius;
    QComboBox* texture_Method;
    QComboBox* texture_Filter;
    QComboBox* sample_Direction;
    QPushButton* generate;
    QFormLayout* inputs;
    QVBoxLayout* user_input;
    QImage newImage;

    int currentHighlightedIndex;
};
