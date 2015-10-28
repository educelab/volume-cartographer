//----------------------------------------------------------------------------------------------------------------------------------------
// Global_Values.h file for Global_Values Class
// Purpose: Used to pass values through the program
// Developer: Michael Royal - mgro224@g.uky.edu
// October 12, 2015 - Spring Semester 2016
// Last Updated 10/23/2015 by: Michael Royal
//----------------------------------------------------------------------------------------------------------------------------------------

#include <QApplication>
#include <QRect>
#include "volumepkg.h"
#include "Texture_Viewer.h"
#include "Segmentations_Viewer.h"


#ifndef VC_DEFAULTVALUES_H
#define VC_DEFAULTVALUES_H


class Global_Values
{

public:
    Global_Values(QRect rec);
    int getHeight();
    int getWidth();
    void setPath(QString newPath);
    void createVolumePackage();
    void getMySegmentations();
    std::vector<std::string> getSegmentations();
    void setTexture_Viewer(Texture_Viewer * texture_Viewer);
    void setSegmentations_Viewer(Segmentations_Viewer *segmentations_Viewer);

private:

    int height;
    int width;
    QString path;
    VolumePkg *vpkg;
    std::vector<std::string> segmentations;

    Texture_Viewer *_texture_Viewer;
    Segmentations_Viewer *_segmentations_Viewer;


};
#endif //VC_DEFAULTVALUES_H
