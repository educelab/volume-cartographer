//----------------------------------------------------------------------------------------------------------------------------------------
// Segmentations_Viewer.cpp file for Segmentations_Viewer Class
// Purpose: Create Segmentations_Viewer Class
// Developer: Michael Royal - mgro224@g.uky.edu
// October 12, 2015 - Spring Semester 2016
// Last Updated 10/23/2015 by: Michael Royal
//----------------------------------------------------------------------------------------------------------------------------------------

#include "Segmentations_Viewer.h"

Segmentations_Viewer::Segmentations_Viewer(Global_Values *globals, Texture_Viewer *texture_Viewer)
{
    _globals = globals;
    _texture_Viewer = texture_Viewer;

    //RIGHT SIDE OF GUI
    //********************************************************************************************

    panels = new QVBoxLayout();

    //Volume_Package Section
    //----------------------------------------------------------
    volume_Package = new QLabel("Volume_Package");
    segmentations = new QListWidget();
    segmentations->setMaximumSize(350,_globals->getHeight());

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

    texture_Method->addItem("Intersection"); 
    texture_Method->addItem("Non-Maximum Suppression"); 
    texture_Method->addItem("Maximum"); 
    texture_Method->addItem("Minimum"); 
    texture_Method->addItem("Median w/ Averaging"); 
    texture_Method->addItem("Median"); 
    texture_Method->addItem("Mean");

    sample_Direction->addItem("Omni");
    sample_Direction->addItem("Positive");
    sample_Direction->addItem("Negative");

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

    connect(segmentations, SIGNAL(itemClicked(QListWidgetItem *)), this, SLOT(itemClickedSlot()));
    connect(generate, SIGNAL(released()),this, SLOT(generateTextureImage()));

    // END OF RIGHT SIDE OF GUI
    //********************************************************************************************

}// End of Default Constructor()

void Segmentations_Viewer::itemClickedSlot()
{
    QString s = segmentations->currentItem()->text();// Gets a QString for the Current Item Selected
    _globals->getVolPkg()->setActiveSegmentation(s.toStdString());// Sets the active Segmentation

    bool test = loadImage();

    if(test==true)
    {
        _texture_Viewer->setImage();
    }
}

void Segmentations_Viewer::generateTextureImage()
{
    //Implement Code Here
}

bool Segmentations_Viewer::loadImage()
{
    cv::Mat texture = _globals->getVolPkg()->getTextureData();
    if ( texture.data == nullptr )
    {
        std::cout<<"Error"<<std::endl; // Output Warning
        std::cout<<"There is no Current Texture Image"<<std::endl;
        return false;

    } else
    {
        //Convert to QPixMap and Display
        std::cout<<"Working";
        texture.convertTo( texture, CV_8U, 255.0/65535.0);
        cv::cvtColor( texture, texture, CV_GRAY2RGB );

        QImage Image( texture.data, texture.cols, texture.rows, texture.step, QImage::Format_RGB888 );
        newImage = Image;
        _globals->setQPixMapImage(newImage);
        return true;
    }
}

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


