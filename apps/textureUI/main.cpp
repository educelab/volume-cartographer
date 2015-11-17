//-----------------------------------------------------------
// GUI for the Volume Cartographer
// GUI Layout
// Developer: Michael Royal - mgro224@g.uky.edu
// October 7, 2015 - Spring Semester 2016
// Last Updated 10/27/2015 by: Michael Royal
//-----------------------------------------------------------
#include <QtWidgets>
#include <mainwindow.h>

int main(int argc, char **argv)
{
    QApplication application(argc, argv);
    QRect rec = QApplication::desktop()->screenGeometry();

    //Create new Global_Values Object
    Global_Values *passValues = new Global_Values(rec);

    //MAIN WINDOW
    //---------------------------------------------------
    MainWindow *window = new MainWindow(passValues);
    passValues->setWindow(window);

    window->show();

    return application.exec();

}// End of main()