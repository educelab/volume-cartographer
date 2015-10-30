


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
    MainWindow(Global_Values *globals,Texture_Viewer *texture_Viewer, Segmentations_Viewer *segmentations_Viewer );

public slots:
    void getFilePath();
    void showNotes();

private:
    void create_Actions();
    void create_Menus();

    QMenuBar* menu_Bar;
    QMenu* fileMenu;
    QMenu* optionsMenu;
    QAction* actionGetFilePath;
    QAction* actionNotes;

    Global_Values *_globals;
    Texture_Viewer *_texture_Viewer;
    Segmentations_Viewer *_segmentations_Viewer;
};

#endif //VC_TEXTURE_VIEWER_H