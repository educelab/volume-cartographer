

#include <mainwindow.h>
#include <QtWidgets>

MainWindow::MainWindow(Global_Values *globals, Segmentations_Viewer *segmentations_Viewer)
{
    _globals = globals;
    _segmentations_Viewer = segmentations_Viewer;

    setWindowTitle("VC_Starter Project");// Set Window Title
    //MAX DIMENSIONS
    window()->setMinimumHeight(_globals->getHeight()/4);
    window()->setMinimumWidth(_globals->getWidth()/4);
    //MIN DIMENSIONS
    window()->setMaximumHeight(_globals->getHeight());
    window()->setMaximumWidth(_globals->getWidth());
    //----------------------------------------------------

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
            QMessageBox::information(this, tr("File Path"), filename);
            _globals->setPath(filename);
            _globals->createVolumePackage();
            _globals->getMySegmentations();
            _segmentations_Viewer->setSegmentations();

        } else {
                    QMessageBox::warning(this, tr("Error Message"), "Invalid File.");
                    //std::cout << "INVALID FILE!!!";
               }
    }
}

void MainWindow::save()
{
    std::cout<<"Saving.........";
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

    optionsMenu = new QMenu("Options");
    optionsMenu->addAction(actionSave);
    menu_Bar->addMenu(optionsMenu);
}
