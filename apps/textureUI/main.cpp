//----------------------------------------------------------------------------------------------------------------------------------------
// GUI for the Volume Cartographer
// GUI Layout
// Developer: Michael Royal - mgro224@g.uky.edu
// October 7, 2015 - Spring Semester 2016
// Last Updated 10/27/2015 by: Michael Royal

// Copy Right ©2015 (Brent Seales: Volume Cartography Research) - University of Kentucky Center for Visualization and Virtualization
//----------------------------------------------------------------------------------------------------------------------------------------

#include <mainwindow.h>
#include <QtWidgets>

int main(int argc, char **argv)
{
    QApplication application(argc, argv);// Creates a new Event Loop
    QRect rec = QApplication::desktop()->screenGeometry();// Instantiates a QRec "Rectangle" Object and gives it the dimensions of the screen display (this will be passed to Global Values Object to enable the GUI to make decisions about default display.

    //Create new Global_Values Object
    Global_Values *passValues = new Global_Values(rec);// Creates an Object "Global_Values" and passes in rec to give the "Global_Values" Object the screen dimensions.
    //Create new Texture_Viewer Object (Left Side of GUI Display)
    Texture_Viewer *texture_Image = new Texture_Viewer(passValues);// Creates an Object "Texture_Viewer" and passes in a "Global_Values" Object named "*passValues" so that the ["Texture Viewer" Object] has access to the ["Global_Values" Object].
    //Create new Segmentations_Viewer Object (Right Side of GUI Display)
    Segmentations_Viewer *segmentations = new Segmentations_Viewer(passValues, texture_Image);// Creates an Object "Segmentations_Viewer" and passes in 2 pointers which enables "*segmentations" to have access/modify data in the "Global_Values" and "Texture_Viewer" Objects

    //MAIN WINDOW Attributes
    //-----------------------------------------------------------------------------------------------------------------------------------------------------------------------

    // Creates a MainWindow Object aka: The GUI Window,
    // and passes in 2 pointers (*passValues, a "Global_Values" Object, and *segmentations, a "Segmentations_Viewer" Object)
    // Note: "Segmentations_Viewer" Object has access to the "Texture_Viewer" Object, Therefore, the GUI MainWindow has access to display/modify data in these Objects
    MainWindow *window = new MainWindow(passValues, segmentations);

    //Sets *window in "Global_Values" to the MainWindow/GUI
    passValues->setWindow(window);

    QHBoxLayout *mainLayout = new QHBoxLayout();// Creates a QHBoxLayout *pointer. The QHBoxLayout allows the developer to add layouts to the mainLayout and show these horizontally. (The use of QHBoxLayout was specifically chosen for the requested user Viewing Experience.)
    mainLayout->addLayout(texture_Image->getLayout());// texture_Image->getLayout() returns the "Image_Management" Layout (Left Side of Screen/GUI) -> THIS LAYOUT HOLDS THE WIDGETS FOR THE OBJECT "Texture_Viewer" which Enables the user to view images, zoom in, zoom out, and reset the image.
    mainLayout->addLayout(segmentations->getLayout());// segmentations->getLayout() returns the "panels" Layout (Right Side of Screen/GUI) -> THIS LAYOUT HOLDS THE WIDGETS FOR THE OBJECT "Segmentations_Viewer" which Enables the user to load segmentations, and generate new texture images.

    QWidget *w = new QWidget();// Creates the Primary Widget to display GUI Functionality
    w->setLayout(mainLayout);// w(the main window) gets assigned the mainLayout
    //-----------------------------------------------------------------------------------------------------------------------------------------------------------------------

    // Display the Main Window
    //---------------------------------------------------------------------------------------------------------------------------------
    window->setCentralWidget(w);// Sets the primary widget "w" as the main widget which "Houses" all the Widgets Via the mainLayout
    window->show();// Shows the Main GUI Window
    //---------------------------------------------------------------------------------------------------------------------------------

    return application.exec();// Returns the Event loop

}// End of main()