// GUI for the Volume Cartographer
// GUI Layout
// Developer: Michael Royal - mgro224@g.uky.edu
// October 7, 2015 - Spring Semester 2016
// Last Updated 11/23/2015 by: Michael Royal

// Copyright 2015 (Brent Seales: Volume Cartography Research)
// University of Kentucky VisCenter
#include <QtWidgets>
#include "MainWindow.hpp"

int main(int argc, char** argv)
{
    QApplication application(argc, argv);
    QRect rec = QApplication::desktop()->screenGeometry();

    // Create new GlobalValues Object
    GlobalValues* passValues = new GlobalValues(rec);

    // Creates a MainWindow Object aka: The GUI Window,
    // and passes in a pointer to the global values
    MainWindow* window = new MainWindow(passValues);

    // Sets *window in "GlobalValues" to the MainWindow/GUI
    passValues->setWindow(window);

    window->show();

    return application.exec();  // Returns the Event loop

}  // End of main()
