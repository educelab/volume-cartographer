//-----------------------------------------------------------
// GUI for the Volume Cartographer
// GUI Layout
// Developer: Michael Royal - mgro224@g.uky.edu
// October 7, 2015 - Spring Semester 2016
// Last Updated 10/16/2015 by: Michael Royal
//-----------------------------------------------------------

#include <QApplication>
#include <QtWidgets>
#include "Texture_Viewer.h"


int main(int argc, char* argv[])
{
    QApplication application(argc, argv);// Creates Event Loop()
    Texture_Viewer *texture_Image = new Texture_Viewer();

    // GETS THE MONITOR'S SCREEN Length & Width
    QRect rec = QApplication::desktop()->screenGeometry();
    int height = rec.height();
    int width = rec.width();

    // GRAPHICAL USER INTERFACE LAYOUT && FORMATTING
    //---------------------------------------------------------------------

    //RIGHT SIDE OF GUI
    //********************************************************************************************

    QVBoxLayout *panels = new QVBoxLayout();

    //Volume_Package Section
    //----------------------------------------------------------
    QLabel *volume_Package = new QLabel("Volume_Package");

    QListWidget *segmentations = new QListWidget();
    segmentations->setMaximumSize(250,height);
    panels->addWidget(volume_Package);
    panels->addWidget(segmentations);
    //End of Volume_Package Section
    //-----------------------------------------------------------

    //Parameters Section
    //-------------------------------------------------------------

    QLabel *parameters = new QLabel("Parameters");
    QSpinBox *radius = new QSpinBox();
    QComboBox *texture_Method = new QComboBox();
    QComboBox *sample_Direction = new QComboBox();
    QPushButton * generate = new QPushButton("Generate Texture");

    QFormLayout *inputs = new QFormLayout();
    inputs->addRow("Radius: (Voxels)", radius);
    inputs->addRow("Texture Method:", texture_Method);
    inputs->addRow("Sample Direction:", sample_Direction);

    QVBoxLayout *user_input = new QVBoxLayout();
    user_input->addWidget(parameters);
    user_input->addLayout(inputs);
    user_input->addWidget(generate);

    panels->addLayout(user_input);

    //End of Parameters Section
    //-------------------------------------------------------------

    // END OF RIGHT SIDE OF GUI
    //********************************************************************************************

    QHBoxLayout *mainLayout = new QHBoxLayout();
    mainLayout->addLayout(texture_Image->getLayout());// Adds Image_Management Layout (Left Side of Screen)
    mainLayout->addLayout(panels);


    QWidget *w = new QWidget();// Creates the Primary Widget to display GUI Functionality
    w->setLayout(mainLayout);// w(the main window) gets assigned the mainLayout

    // Window Title
    w->setWindowTitle("VC_Starter Project");// Sets the title for the main window
    w->setMinimumHeight(height/4);
    w->setMinimumWidth(height/4);
    w->setMaximumHeight(height);
    w->setMaximumWidth(width);

    // Display Window
    w->show();

    //----------------------------------------------------------------------
    //---------------------------------------------------- END OF GUI LAYOUT

    //Event Loop()
    return application.exec();// Executes the Application/Program


}// End of main()