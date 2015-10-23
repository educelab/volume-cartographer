//----------------------------------------------------------------------------------------------------------------------------------------
// DefaultValues.cpp file for DefaultValues Class
// Purpose: Implements DefaultValues Class
// Developer: Michael Royal - mgro224@g.uky.edu
// October 12, 2015 - Spring Semester 2016
// Last Updated 10/23/2015 by: Michael Royal
//----------------------------------------------------------------------------------------------------------------------------------------

#include "DefaultValues.h"


DefaultValues::DefaultValues(QRect rec)
{
    // GETS THE MONITOR'S SCREEN Length & Width
    height = rec.height();
    width = rec.width();

}// End of Default Constructor()

int DefaultValues::getHeight()
{
    return height;
}

int DefaultValues::getWidth()
{
    return width;
}