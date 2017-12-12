// SegmentationsViewer.h file for SegmentationsViewer Class
// Purpose: Create header file for SegmentationsViewer Class
// Developer: Michael Royal - mgro224@g.uky.edu
// October 12, 2015 - Spring Semester 2016
// Last Updated 09/26/2016 by: Michael Royal

// Copyright 2015 (Brent Seales: Volume Cartography Research)
// University of Kentucky VisCenter

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
    Q_OBJECT

public:
    SegmentationsViewer(GlobalValues* globals, TextureViewer* texture_Viewer);
    QVBoxLayout* getLayout();
    void clearGUI();
    void setSegmentations();
    bool loadImage(cv::Mat texture);
    void setVolPkgLabel(QString name);

public slots:
    void itemClicked();
    void generateTexture();
    void setEnabled(bool value);

private:
    MyThread* procThread_;
    GlobalValues* globals_;
    TextureViewer* textureViewer_;

    QVBoxLayout* panels_;
    QLabel* volPkgLabel_;
    QListWidget* segList_;
    QString activeSegID_;

    QLabel* paramsLabel_;
    QSpinBox* radiusSelect_;
    QComboBox* methodSelect_;
    QComboBox* filterSelect_;
    QComboBox* directionSelect_;

    QPushButton* startGen_;
    QFormLayout* paramsContainer_;
    QVBoxLayout* paramsSubcontainer_;

    QImage displayImage_;

    int activeSegIndex_;
};
