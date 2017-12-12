// TextureViewer.h file for TextureViewer Class
// Purpose: Create header file for TextureViewer Class
// Developer: Michael Royal - mgro224@g.uky.edu
// October 12, 2015 - Spring Semester 2016
// Last Updated 10/16/2015 by: Michael Royal
// Code Reference:
// http://doc.qt.io/qt-5/qtwidgets-widgets-imageviewer-example.html ---(I
// edited/formatted the code to suit our purposes)

// Copyright 2015 (Brent Seales: Volume Cartography Research)
// University of Kentucky VisCenter

#pragma once

#include <QAction>
#include <QLabel>
#include <QLayout>
#include <QMenu>
#include <QMenuBar>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollArea>
#include <QScrollBar>
#include <qmainwindow.h>
#include <qnumeric.h>
#include "GlobalValues.hpp"

class TextureViewer : QObject
{
    Q_OBJECT

public:
    explicit TextureViewer(GlobalValues* globals);
    QVBoxLayout* getLayout();
    void clearGUI();
    void loadImageFromGlobals();
    void clearImageArea();
    void setProgressActive(bool value);
    void setEnabled(bool value);
    void clearLabel();

private slots:
    void zoom_in_();
    void zoom_out_();
    void zoom_reset_();

private:
    void setup_actions_();
    void scale_image_(double factor);
    void adjust_scroll_bar_(QScrollBar* scrollBar, double factor);

    GlobalValues* globals_;

    QPushButton* zoomIn_;
    QPushButton* zoomOut_;
    QPushButton* refresh_;
    QLabel* spacer_;
    QLabel* viewer_;
    QLabel* imageLabel_;
    QScrollArea* scrollArea_;
    QHBoxLayout* zoomPanel_;
    QVBoxLayout* viewerPanel_;
    QProgressBar* progressBar_;
    QPixmap pix_;
    QAction* zoomInAction_;
    QAction* zoomOutAction_;
    QAction* zoomResetAction_;

    double scaleFactor_{1};
};
