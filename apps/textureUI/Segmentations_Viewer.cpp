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

    cv::Mat texture = _globals->getVolPkg()->getTextureData();

    bool test = loadImage(texture);

    if(test==true)
    {
        _texture_Viewer->setImage();

    }else _texture_Viewer->clearImageLabel();
}

void Segmentations_Viewer::generateTextureImage()
{
    if(_globals->isVPKG_Intantiated() && _globals->getSegmentations().size()>0)
    {
        QMessageBox::information(_globals->getWindow(), "Generating", "Please click \"OK\" to begin Generating your Texture Image.");

        double _radius = radius->text().toDouble();
        int meshWidth = -1;
        int meshHeight = -1;

        std::string meshName = _globals->getVolPkg()->getMeshPath();

        VC_Composite_Option aFilterOption = ( VC_Composite_Option )texture_Method->currentIndex();
        VC_Direction_Option aDirectionOption = ( VC_Direction_Option )sample_Direction->currentIndex();

        // declare pointer to new Mesh object
        VC_MeshType::Pointer  mesh = VC_MeshType::New();

        // try to convert the ply to an ITK mesh
        if (!volcart::io::ply2itkmesh(meshName, mesh, meshWidth, meshHeight))
        {
            exit( -1 );
        };

        volcart::Texture newTexture;
        newTexture = volcart::texturing::compositeTexture( mesh, *_globals->getVolPkg(), meshWidth, meshHeight, _radius, aFilterOption, aDirectionOption );

        // Display this. This is a 16-bit, single channel image.
        cv::Mat texture = newTexture.getImage(0).clone();

        bool test = loadImage(texture);

        if(test == true)
        {
            _texture_Viewer->setImage();
            QMessageBox::warning(_globals->getWindow(),"Warning", "The Generated Image is not Saved, if you wish to save it, please select \"Options\" -> \"Save Texture\".");
        }

    }else QMessageBox::warning(_globals->getWindow(),"Error", "No Segmentation has been loaded, Please load Segmentation.");

}

bool Segmentations_Viewer::loadImage(cv::Mat texture)
{
    cv::Mat _texture = texture;
    if ( texture.data == nullptr )
    {
        _texture_Viewer->clearImageLabel();
        QMessageBox::warning(_globals->getWindow(), "Error", "There is no Current Texture Image");
        return false;

    } else
    {
        //Convert to QPixMap and Display
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


    if(_globals->getSegmentations().size()>0)// Loads first Image to Screen by default
    {
        segmentations->setCurrentRow(0);
        itemClickedSlot();
    }

}


