//----------------------------------------------------------------------------------------------------------------------------------------
// DefaultValues.h file for DefaultValues Class
// Purpose: Used to pass values through the program
// Developer: Michael Royal - mgro224@g.uky.edu
// October 12, 2015 - Spring Semester 2016
// Last Updated 10/23/2015 by: Michael Royal
//----------------------------------------------------------------------------------------------------------------------------------------

#include <QApplication>
#include <QRect>


#ifndef VC_DEFAULTVALUES_H
#define VC_DEFAULTVALUES_H


class DefaultValues
{

public:
    DefaultValues(QRect rec);
    int getHeight();
    int getWidth();

private:

    int height;
    int width;


};


#endif //VC_DEFAULTVALUES_H
