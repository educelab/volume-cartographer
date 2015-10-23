//-----------------------------------------------------------
// GUI for the Volume Cartographer
// GUI Layout
// Developer: Michael Royal - mgro224@g.uky.edu
// October 7, 2015 - Spring Semester 2016
// Last Updated 10/23/2015 by: Michael Royal
//-----------------------------------------------------------

#include <QApplication>
#include <QtWidgets>
#include "Texture_Viewer.h"
#include "Segmentations_Viewer.h"


int main(int argc, char* argv[])
{
    QApplication application(argc, argv);// Creates Event Loop()

    QRect rec = QApplication::desktop()->screenGeometry();

    //Create new DefaultValues Object
    DefaultValues *passValues = new DefaultValues(rec);
    //Create new Texture_Viewer Object
    Texture_Viewer *texture_Image = new Texture_Viewer(passValues);
    //Create new Segmentations_Viewer Object
    Segmentations_Viewer *segmentations = new Segmentations_Viewer(passValues);

    // GRAPHICAL USER INTERFACE LAYOUT && FORMATTING
    //---------------------------------------------------------------------

    QHBoxLayout *mainLayout = new QHBoxLayout();
    mainLayout->addLayout(texture_Image->getLayout());// Adds Image_Management Layout (Left Side of Screen)
    mainLayout->addLayout(segmentations->getLayout());

    QWidget *w = new QWidget();// Creates the Primary Widget to display GUI Functionality
    w->setLayout(mainLayout);// w(the main window) gets assigned the mainLayout

    // Window Title
    w->setWindowTitle("VC_Starter Project");// Sets the title for the main window

    //MAX DIMENSIONS
    w->setMinimumHeight(passValues->getHeight()/4);
    w->setMinimumWidth(passValues->getWidth()/4);

    //MIN DIMENSIONS
    w->setMaximumHeight(passValues->getHeight());
    w->setMaximumWidth(passValues->getWidth());

    // Display Window
    w->show();
    //----------------------------------------------------------------------
    //---------------------------------------------------- END OF GUI LAYOUT

    //Event Loop()
    return application.exec();// Executes the Application/Program


}// End of main()