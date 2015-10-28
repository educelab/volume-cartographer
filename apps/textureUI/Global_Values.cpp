//----------------------------------------------------------------------------------------------------------------------------------------
// Global_Values.cpp file for Global_Values Class
// Purpose: Implements Global_Values Class
// Developer: Michael Royal - mgro224@g.uky.edu
// October 12, 2015 - Spring Semester 2016
// Last Updated 10/23/2015 by: Michael Royal
//----------------------------------------------------------------------------------------------------------------------------------------

#include "Global_Values.h"

Global_Values::Global_Values(QRect rec)
{
    // GETS THE MONITOR'S SCREEN Length & Width
    height = rec.height();
    width = rec.width();

}// End of Default Constructor()

int Global_Values::getHeight()
{
    return height;
}

int Global_Values::getWidth()
{
    return width;
}

void Global_Values::setPath(QString newPath)
{
    path = newPath;
}

void Global_Values::createVolumePackage()
{
    *vpkg = VolumePkg(path.toStdString());// Creates a Volume Package
}

void Global_Values::getMySegmentations()
{
    segmentations = vpkg->getSegmentations();
    for(int i=0; i<segmentations.size(); i++)
    {
        std::cout<<segmentations[i]<<"   ";
    }
}