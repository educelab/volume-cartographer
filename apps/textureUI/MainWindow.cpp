#include <mainwindow.h>

MainWindow::MainWindow(Global_Values *globals)
{
    _globals = globals;

    setWindowTitle("VC Texture");// Set Window Title
    //MAX DIMENSIONS
    window()->setMinimumHeight(_globals->getHeight()/3);
    window()->setMinimumWidth(_globals->getWidth()/3);
    //MIN DIMENSIONS
    window()->setMaximumHeight(_globals->getHeight());
    window()->setMaximumWidth(_globals->getWidth());
    //----------------------------------------------------

    //Create new Texture_Viewer Object
    Texture_Viewer *texture_Image = new Texture_Viewer(globals);
    //Create new Segmentations_Viewer Object
    Segmentations_Viewer *segmentations = new Segmentations_Viewer(globals, texture_Image);
    _segmentations_Viewer = segmentations;

    QHBoxLayout *mainLayout = new QHBoxLayout();
    mainLayout->addLayout(texture_Image->getLayout());// Adds Image_Management Layout (Left Side of Screen)
    mainLayout->addLayout(segmentations->getLayout());

    QWidget *w = new QWidget();// Creates the Primary Widget to display GUI Functionality
    w->setLayout(mainLayout);// w(the main window) gets assigned the mainLayout

    // Display Window
    //------------------------------
    setCentralWidget(w);

    create_Actions();
    create_Menus();
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
    connect( actionGetFilePath, SIGNAL( triggered() ), this, SLOT( getFilePath() ) );

    actionSave = new QAction( "Save Texture", this );
    connect( actionSave, SIGNAL( triggered() ), this, SLOT( save() ) );
}

void MainWindow::create_Menus()
{
    fileMenu = new QMenu( tr( "&File" ), this );
    fileMenu->addAction(actionGetFilePath);
    fileMenu->addAction(actionSave);

    menuBar()->addMenu(fileMenu);
}
