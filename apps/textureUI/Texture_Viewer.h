//----------------------------------------------------------------------------------------------------------------------------------------
// Texture_Viewer.h file for Texture_Viewer Class
// Purpose: Create header file for Texture_Viewer Class
// Developer: Michael Royal - mgro224@g.uky.edu
// October 12, 2015 - Spring Semester 2016
// Last Updated 10/16/2015 by: Michael Royal
// Code Reference: http://doc.qt.io/qt-5/qtwidgets-widgets-imageviewer-example.html ---(I edited/formatted the code to suit our purposes)
//----------------------------------------------------------------------------------------------------------------------------------------

#ifndef VC_TEXTURE_VIEWER_H
#define VC_TEXTURE_VIEWER_H

#include <qmainwindow.h>
#include <QScrollBar>
#include <QLabel>
#include <QScrollArea>
#include <qnumeric.h>
#include <QAction>
#include <QPushButton>
#include <QLayout>
#include "Global_Values.h"
#include <QAction>
#include <QMenuBar>
#include <QMenu>

class Texture_Viewer:QObject
{
    Q_OBJECT

public:
    Texture_Viewer(Global_Values *globals);
    QVBoxLayout * getLayout();
    void setImage();
    void clearImageLabel();

private slots:
    void open();
    void zoom_In();
    void zoom_Out();
    void reset_Size();

private:
    void create_Actions();
    void scale_Texture(double factor);
    void adjustScrollBar(QScrollBar *scrollBar, double factor);

    QPushButton *zoomIn;
    QPushButton *zoomOut;
    QPushButton *refresh;
    QLabel *spacer;
    QLabel *viewer;
    QLabel *imageLabel;
    QLabel *temp;
    QScrollArea *scrollArea;
    QHBoxLayout *zoom;
    QVBoxLayout *image_Management;

    QPixmap pix;

    double scaleFactor;

    Global_Values *_globals;

    QAction *zoomInAction;
    QAction *zoomOutAction;
    QAction *resetSizeAction;

};

#endif //VC_TEXTURE_VIEWER_H
