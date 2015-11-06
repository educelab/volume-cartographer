//----------------------------------------------------------------------------------------------------------------------------------------
// Global_Values.h file for Global_Values Class
// Purpose: Used to pass values through the program
// Developer: Michael Royal - mgro224@g.uky.edu
// October 12, 2015 - Spring Semester 2016
// Last Updated 10/23/2015 by: Michael Royal
//----------------------------------------------------------------------------------------------------------------------------------------

#ifndef VC_DEFAULTVALUES_H
#define VC_DEFAULTVALUES_H

#include <QApplication>
#include <QRect>
#include "volumepkg.h"
#include <QImage>
#include <QPixmap>
#include <QMainWindow>

class Global_Values
{

public:
    Global_Values(QRect rec);
    int getHeight();
    int getWidth();
    VolumePkg *getVolPkg();
    void setPath(QString newPath);
    void createVolumePackage();
    void getMySegmentations();
    std::vector<std::string> getSegmentations();
    void setQPixMapImage(QImage image);
    QPixmap getQPixMapImage();
    bool isVPKG_Intantiated();
    void setWindow(QMainWindow *window);
    QMainWindow *getWindow();

private:

    bool VPKG_Instantiated = false;
    int height;
    int width;
    QString path;
    VolumePkg *vpkg;
    std::vector<std::string> segmentations;
    QPixmap pix;
    QMainWindow *_window;


};
#endif //VC_DEFAULTVALUES_H
