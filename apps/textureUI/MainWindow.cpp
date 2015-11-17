//----------------------------------------------------------------------------------------------------------------------------------------
// MainWindow.cpp file for MainWindow Class , (Implements QMainWindow)
// Purpose: Create a Main Window for the GUI
// Developer: Michael Royal - mgro224@g.uky.edu
// October 12, 2015 - Spring Semester 2016
// Last Updated 11/13/2015 by: Michael Royal

// Copy Right ©2015 (Brent Seales: Volume Cartography Research) - University of Kentucky Center for Visualization and Virtualization
//----------------------------------------------------------------------------------------------------------------------------------------

#include <mainwindow.h>

MainWindow::MainWindow(Global_Values *globals)
{
    _globals = globals; // Enables access to Global Values Object

    setWindowTitle("VC Texture");// Set Window Title

    //NOTE: Minimum Height and Width -------------------------
    // will be different on other display screens,
    // if Resolution is too small may cause distortion
    // of Buttons Visually when Program first Initiates
    //----------------------------------------------------------

    //MAX DIMENSIONS
    window()->setMinimumHeight(_globals->getHeight()/2);
    window()->setMinimumWidth(_globals->getWidth()/2);
    //MIN DIMENSIONS
    window()->setMaximumHeight(_globals->getHeight());
    window()->setMaximumWidth(_globals->getWidth());
    //---------------------------------------------------------

    //Create new Texture_Viewer Object (Left Side of GUI Display)
    Texture_Viewer *texture_Image = new Texture_Viewer(globals);
    //Create new Segmentations_Viewer Object (Right Side of GUI Display)
    Segmentations_Viewer *segmentations = new Segmentations_Viewer(globals, texture_Image);
    _segmentations_Viewer = segmentations;

    QHBoxLayout *mainLayout = new QHBoxLayout();
    mainLayout->addLayout(texture_Image->getLayout()); // THIS LAYOUT HOLDS THE WIDGETS FOR THE OBJECT "Texture_Viewer" which Enables the user to view images, zoom in, zoom out, and reset the image.
    mainLayout->addLayout(segmentations->getLayout()); // THIS LAYOUT HOLDS THE WIDGETS FOR THE OBJECT "Segmentations_Viewer" which Enables the user to load segmentations, and generate new texture images.

    QWidget *w = new QWidget();// Creates the Primary Widget to display GUI Functionality
    w->setLayout(mainLayout);// w(the main window) gets assigned the mainLayout

    // Display Window
    //------------------------------
    setCentralWidget(w); // w is a wrapper widget for all of the widgets in the main window.

    create_Actions(); // Creates the Actions for the File Menu
    create_Menus(); // Creates the Menus and adds them to the Menu Bar
}

void MainWindow::getFilePath()
{
    QFileDialog *dialogBox= new QFileDialog();
    QString filename = dialogBox->getExistingDirectory();
    std::string file_Name = filename.toStdString();

    if(filename!=NULL)
    {
        if ((file_Name.substr(file_Name.length()-7, file_Name.length())).compare(".volpkg") == 0)
        {
            try {
                    _globals->setPath(filename);
                    _globals->createVolumePackage();
                    _globals->getMySegmentations();
                    _segmentations_Viewer->setSegmentations();
                    _segmentations_Viewer->setVol_Package_Name(filename);

                }catch(...)
                        {
                            QMessageBox::warning(this, tr("Error Message"), "Error Opening File.");
                        };


        } else {
                    QMessageBox::warning(this, tr("Error Message"), "Invalid File.");
               }
    }
}

void MainWindow::save() // Need a try catch for failure
{
    // NEEDS TO BE CONFIGURED
    QString imagePath = QFileDialog::getSaveFileName(this, tr("Save File"), "", "PNG (*.png)");
    // Needs to be configured
    _globals->getQPixMapImage().save(imagePath, "PNG");
}

void MainWindow::create_Actions()
{
    actionGetFilePath = new QAction( "Open volume package...", this );
    connect( actionGetFilePath, SIGNAL( triggered() ), this, SLOT( getFilePath() ) );

    actionSave = new QAction( "Export Texture", this );
    connect( actionSave, SIGNAL( triggered() ), this, SLOT( save() ) );
}

void MainWindow::create_Menus()
{
    fileMenu = new QMenu( tr( "&File" ), this );
    fileMenu->addAction(actionGetFilePath);
    fileMenu->addAction(actionSave);

    menuBar()->addMenu(fileMenu);
}
