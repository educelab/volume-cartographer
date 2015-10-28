//----------------------------------------------------------------------------------------------------------------------------------------
// Segmentations_Viewer.cpp file for Segmentations_Viewer Class
// Purpose: Create Segmentations_Viewer Class
// Developer: Michael Royal - mgro224@g.uky.edu
// October 12, 2015 - Spring Semester 2016
// Last Updated 10/23/2015 by: Michael Royal
//----------------------------------------------------------------------------------------------------------------------------------------

#include "Segmentations_Viewer.h"

Segmentations_Viewer::Segmentations_Viewer(Global_Values *globals)
{
    _globals = globals;

    //RIGHT SIDE OF GUI
    //********************************************************************************************

    panels = new QVBoxLayout();

    //Volume_Package Section
    //----------------------------------------------------------
    volume_Package = new QLabel("Volume_Package");
    segmentations = new QListWidget();
    segmentations->setMaximumSize(250,_globals->getHeight());

    panels->addWidget(volume_Package);
    panels->addWidget(segmentations);
    //End of Volume_Package Section
    //-----------------------------------------------------------

    //Parameters Section
    //-------------------------------------------------------------

    parameters = new QLabel("Parameters");
    radius = new QSpinBox();
    texture_Method = new QComboBox();
    sample_Direction = new QComboBox();
    generate = new QPushButton("Generate Texture");

    inputs = new QFormLayout();
    inputs->addRow("Radius: (Voxels)", radius);
    inputs->addRow("Texture Method:", texture_Method);
    inputs->addRow("Sample Direction:", sample_Direction);

    user_input = new QVBoxLayout();
    user_input->addWidget(parameters);
    user_input->addLayout(inputs);
    user_input->addWidget(generate);

    panels->addLayout(user_input);

    //End of Parameters Section
    //-------------------------------------------------------------

    // END OF RIGHT SIDE OF GUI
    //********************************************************************************************

}// End of Default Constructor()

QVBoxLayout * Segmentations_Viewer::getLayout()
{
    return panels;
}

void Segmentations_Viewer::setSegmentations()
{
    segmentations->clear();

    std::vector<std::string> segments = _globals->getSegmentations();
    QString qstr;

    for(int i=0; i<_globals->getSegmentations().size(); i++)
    {
        qstr = QString::fromStdString(segments[i]);
        segmentations->addItem(qstr);
    }
}