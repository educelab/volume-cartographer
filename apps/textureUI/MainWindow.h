


#ifndef VC_MAINWINDOW_H
#define VC_MAINWINDOW_H

#include <QApplication>
#include <QtWidgets>
#include <QObject>
#include "Global_Values.h"
#include "Texture_Viewer.h"
#include "Segmentations_Viewer.h"

class MainWindow : public QMainWindow
{
    // NOTICE THIS MACRO
    Q_OBJECT
    //

public:
    MainWindow(Global_Values *globals);

public slots:
    void getFilePath();
    void save();

private:
    void create_Actions();
    void create_Menus();

    QMenu* fileMenu;

    QAction* actionGetFilePath;
    QAction* actionSave;

    Global_Values *_globals;
    Segmentations_Viewer *_segmentations_Viewer;
};

#endif //VC_TEXTURE_VIEWER_H