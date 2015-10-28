

#include <mainwindow.h>
#include <QtWidgets>

MainWindow::MainWindow(Global_Values *globals)
{
    _globals = globals;

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

    if(filename!=NULL)
    {
        QMessageBox::information(this, tr("File Path"), filename);
        _globals->setPath(filename);
        _globals->createVolumePackage();
        _globals->getMySegmentations();
        //_globals(segmenations)->setSegmentations();

    }
}

void MainWindow::showNotes()
{
    std::cout<<"NOTES********";
}

void MainWindow::create_Actions()
{
    actionGetFilePath = new QAction( "Load VC_Volume Package", this );
    actionNotes = new QAction( "Notes", this );

    connect( actionGetFilePath, SIGNAL( triggered() ), this, SLOT( getFilePath() ) );

    connect( actionNotes, SIGNAL( triggered() ), this, SLOT( showNotes() ) );
}

void MainWindow::create_Menus()
{
    menu_Bar = new QMenuBar( this );

    fileMenu = new QMenu("File");
    fileMenu->addAction(actionGetFilePath);
    menu_Bar->addMenu(fileMenu);

    optionsMenu = new QMenu("Options");
    optionsMenu->addAction(actionNotes);
    menu_Bar->addMenu(optionsMenu);
}
