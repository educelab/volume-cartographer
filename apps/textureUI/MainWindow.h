//----------------------------------------------------------------------------------------------------------------------------------------
// MainWindow.h file for MainWindow Class , (Implements QMainWindow)
// Purpose: Create a Main Window for the GUI
// Developer: Michael Royal - mgro224@g.uky.edu
// October 12, 2015 - Spring Semester 2016
// Last Updated 11/13/2015 by: Michael Royal
//----------------------------------------------------------------------------------------------------------------------------------------

#ifndef VC_MAINWINDOW_H
#define VC_MAINWINDOW_H

#include <QApplication>
#include <QtGui>
#include <QObject>
#include <QMainWindow>
#include <QMenu>
#include <QAction>
#include <QMenuBar>
#include "Global_Values.h"
#include "Texture_Viewer.h"
#include "Segmentations_Viewer.h"

class MainWindow : public QMainWindow
{
    // NOTICE THIS MACRO
    Q_OBJECT
    //

public:

    // Creates a Constructor for the MainWindow Class that takes in The Global_Values as a *pointer and Segmentations_Viewer as a *pointer so that it can have access to these Objects.
    MainWindow(Global_Values *globals, Segmentations_Viewer *segmentations_Viewer );

public slots:
    void getFilePath();
    void save();

private:
    void create_Actions();
    void create_Menus();

    QMenuBar* menu_Bar;
    QMenu* fileMenu;
    QMenu* optionsMenu;
    QAction* actionGetFilePath;
    QAction* actionSave;

    Global_Values *_globals;
    Segmentations_Viewer *_segmentations_Viewer;
};

#endif //VC_TEXTURE_VIEWER_H