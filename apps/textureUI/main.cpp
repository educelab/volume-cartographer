//-----------------------------------------------------------
// GUI for the Volume Cartographer
// GUI Layout
// Developer: Michael Royal - mgro224@g.uky.edu
// October 7, 2015 - Spring Semester 2016
// Last Updated 10/27/2015 by: Michael Royal
//-----------------------------------------------------------
#include <mainwindow.h>
#include <QtWidgets>

int main(int argc, char **argv)
{
    QApplication application(argc, argv);
    QRect rec = QApplication::desktop()->screenGeometry();

    //Create new Global_Values Object
    Global_Values *passValues = new Global_Values(rec);
    //Create new Texture_Viewer Object
    Texture_Viewer *texture_Image = new Texture_Viewer(passValues);
    //Create new Segmentations_Viewer Object
    Segmentations_Viewer *segmentations = new Segmentations_Viewer(passValues, texture_Image);

    //MAIN WINDOW
    //---------------------------------------------------
    MainWindow *window = new MainWindow(passValues, texture_Image, segmentations);

    QHBoxLayout *mainLayout = new QHBoxLayout();
    mainLayout->addLayout(texture_Image->getLayout());// Adds Image_Management Layout (Left Side of Screen)
    mainLayout->addLayout(segmentations->getLayout());

    QWidget *w = new QWidget();// Creates the Primary Widget to display GUI Functionality
    w->setLayout(mainLayout);// w(the main window) gets assigned the mainLayout

    // Display Window
    //------------------------------
    window->setCentralWidget(w);
    window->show();

    return application.exec();

}// End of main()