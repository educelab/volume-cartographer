//----------------------------------------------------------------------------------------------------------------------------------------
// Segmentations_Viewer.h file for Segmentations_Viewer Class
// Purpose: Create header file for Segmentations_Viewer Class
// Developer: Michael Royal - mgro224@g.uky.edu
// October 12, 2015 - Spring Semester 2016
// Last Updated 10/23/2015 by: Michael Royal
//----------------------------------------------------------------------------------------------------------------------------------------

#include <QLabel>
#include <QListWidget>
#include "vc_defines.h"
#include "vc_datatypes.h"
#include "volumepkg.h"
#include "io/ply2itk.h"
#include "compositeTexture.h"
#include "Global_Values.h"
#include <QVBoxLayout>
#include <QSpinBox>
#include <QComboBox>
#include <QPushButton>
#include <QFormLayout>


#ifndef VC_SEGMENTATIONS_VIEWER_H
#define VC_SEGMENTATIONS_VIEWER_H


class Segmentations_Viewer:QObject
{

public:
    Segmentations_Viewer(Global_Values *values);
    QVBoxLayout * getLayout();

private:

    QVBoxLayout *panels;// Main Layout for Right Side of GUI

    QLabel *volume_Package;
    QListWidget *segmentations;

    QLabel *parameters;
    Global_Values *defaultVals;
    QSpinBox *radius;
    QComboBox *texture_Method;
    QComboBox *sample_Direction;
    QPushButton *generate;

    QFormLayout *inputs;
    QVBoxLayout *user_input;



};

#endif //VC_SEGMENTATIONS_VIEWER_H
