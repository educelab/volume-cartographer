//----------------------------------------------------------------------------------------------------------------------------------------
// GUI for the Volume Cartographer
// GUI Layout
// Developer: Michael Royal - mgro224@g.uky.edu
// October 7, 2015 - Spring Semester 2016
// Last Updated 10/27/2015 by: Michael Royal

// Copyright ©2015 (Brent Seales: Volume Cartography Research) - University of Kentucky Center for Visualization and Virtualization
//----------------------------------------------------------------------------------------------------------------------------------------
#include <QtWidgets>
#include <mainwindow.h>

int main(int argc, char **argv)
{
    QApplication application(argc, argv);// Creates a new Event Loop
    QRect rec = QApplication::desktop()->screenGeometry();// Instantiates a QRec "Rectangle" Object and gives it the dimensions of the screen display (this will be passed to Global Values Object to enable the GUI to make decisions about default display.

    //Create new Global_Values Object
    Global_Values *passValues = new Global_Values(rec);

    //MAIN WINDOW Attributes
    //-----------------------------------------------------------------------------------------------------------------------------------------------------------------------

    // Creates a MainWindow Object aka: The GUI Window,
    // and passes in a pointer to the global values
    MainWindow *window = new MainWindow(passValues);

    //Sets *window in "Global_Values" to the MainWindow/GUI
    passValues->setWindow(window);

    window->show();

    return application.exec();// Returns the Event loop

}// End of main()