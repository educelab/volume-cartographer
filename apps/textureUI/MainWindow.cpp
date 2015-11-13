//----------------------------------------------------------------------------------------------------------------------------------------
// MainWindow.cpp file for MainWindow Class , (Implements QMainWindow)
// Purpose: Create a Main Window for the GUI
// Developer: Michael Royal - mgro224@g.uky.edu
// October 12, 2015 - Spring Semester 2016
// Last Updated 11/13/2015 by: Michael Royal

// Copy Right ©2015 (Brent Seales: Volume Cartography Research) - University of Kentucky Center for Visualization and Virtualization
//----------------------------------------------------------------------------------------------------------------------------------------

#include <mainwindow.h>
#include <QtWidgets>

MainWindow::MainWindow(Global_Values *globals, Segmentations_Viewer *segmentations_Viewer) // Purpose: MainWindow Constructor
{
    _globals = globals; // Enables access to Global Values Object
    _segmentations_Viewer = segmentations_Viewer; // Enables access to Segmentations Viewer Object

    setWindowTitle("VC_Starter Project");// Set Window Title

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
    actionGetFilePath = new QAction( "Load VC_Volume Package", this );
    actionSave = new QAction( "Save Texture", this );

    connect( actionGetFilePath, SIGNAL( triggered() ), this, SLOT( getFilePath() ) );

    connect( actionSave, SIGNAL( triggered() ), this, SLOT( save() ) );
}

void MainWindow::create_Menus()
{
    menu_Bar = new QMenuBar( this );

    fileMenu = new QMenu("File");
    fileMenu->addAction(actionGetFilePath);
    menu_Bar->addMenu(fileMenu);

    _globals->setFileMenu(fileMenu);

    optionsMenu = new QMenu("Options");
    optionsMenu->addAction(actionSave);
    menu_Bar->addMenu(optionsMenu);

    _globals->setOptionsMenu(optionsMenu);
}
