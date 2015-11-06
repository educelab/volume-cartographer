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
    volume_Package->setMaximumWidth(400);
    segmentations = new QListWidget();
    segmentations->setMaximumSize(400,_globals->getHeight());

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
    _texture_Viewer->clearImageLabel();

    QString s = segmentations->currentItem()->text();// Gets a QString for the Current Item Selected
    _globals->getVolPkg()->setActiveSegmentation(s.toStdString());// Sets the active Segmentation

    cv::Mat texture = _globals->getVolPkg()->getTextureData();

    bool test = loadImage(texture);

    if(test==true)
    {
        _texture_Viewer->setImage();

    }
}

void Segmentations_Viewer::setVol_Package_Name(QString name)
{
    const QChar test = '/';
    const QString _name = name;
    QString file = _name;
    int index = -1;

    for(int i=0; i<_name.length();i++)
    {
        if(_name[i]== test)
        {
            index = i;
        }
    }

    std::string filename = _name.toStdString();

    if(index != -1)
    {
        filename = filename.substr(index+1,filename.length());
    }

    const QString _filename = QString::fromStdString(filename);
    volume_Package->setText("Volume Package: " + _filename );
}

void Segmentations_Viewer::generateTextureImage()
{
    bool cloudProblem = false;

    if(_globals->isVPKG_Intantiated() && _globals->getSegmentations().size()>0)
    {
        _texture_Viewer->progressActive(true);// Show Progress Loading Bar
        QMessageBox::information(_globals->getWindow(), "Generating", "Image is Generating...Click \"OK\" .");// Necessary for the ProgressBar to Show

        //QApplication::changeOverrideCursor(Qt::ArrowCursor);
    }

    try {

            if(_globals->isVPKG_Intantiated() && _globals->getSegmentations().size()>0)
            {
                double _radius = radius->text().toDouble();
                int meshWidth = -1;
                int meshHeight = -1;

                std::string meshName = _globals->getVolPkg()->getMeshPath();

                VC_Composite_Option aFilterOption = (VC_Composite_Option) texture_Method->currentIndex();
                VC_Direction_Option aDirectionOption = (VC_Direction_Option) sample_Direction->currentIndex();

                // declare pointer to new Mesh object
                VC_MeshType::Pointer mesh = VC_MeshType::New();

                // try to convert the ply to an ITK mesh
                if (!volcart::io::ply2itkmesh(meshName, mesh, meshWidth, meshHeight))
                {
                    cloudProblem = true;
                    QMessageBox::warning(_globals->getWindow(),"Error", "Failed to Generate Texture [cloud.ply] error.");
                    throw(__EXCEPTIONS);// Error

                };

                volcart::Texture newTexture;
                newTexture = volcart::texturing::compositeTexture(mesh, *_globals->getVolPkg(), meshWidth, meshHeight, _radius, aFilterOption, aDirectionOption);

                // Display this. This is a 16-bit, single channel image.
                cv::Mat texture = newTexture.getImage(0).clone();

                bool test = loadImage(texture);

                if (test == true)
                {
                    _texture_Viewer->progressActive(false);
                    _texture_Viewer->setImage();
                    QMessageBox::warning(_globals->getWindow(), "Warning", "The Generated Image is not Saved, if you wish to save it, please select \"Options\" then \"Save Texture\".");
                }

            }else QMessageBox::warning(_globals->getWindow(),"Error", "No Segmentation has been loaded, Please load Segmentation.");

    }catch(...)
        {
            if(cloudProblem == false)
            QMessageBox::warning(_globals->getWindow(),"Error", "Failed to Generate Texture.");
        };

}// End of generateTextureImage()

bool Segmentations_Viewer::loadImage(cv::Mat texture)
{
    try
    {
        cv::Mat _texture = texture;
        if (texture.data == nullptr)
        {
            _texture_Viewer->clearImageLabel();
            QMessageBox::warning(_globals->getWindow(), "Error", "There is no Current Texture Image");
            return false;

        } else
            {
                //Convert to QPixMap and Display
                texture.convertTo(texture, CV_8U, 255.0 / 65535.0);
                cv::cvtColor(texture, texture, CV_GRAY2RGB);

                QImage Image(texture.data, texture.cols, texture.rows, texture.step, QImage::Format_RGB888);
                newImage = Image;
                _globals->setQPixMapImage(newImage);
                return true;
            }

    }catch(...)
        {
            QMessageBox::warning(_globals->getWindow(), "Error", "Failed to Load Image Properly");
        }

}

QVBoxLayout * Segmentations_Viewer::getLayout()
{
    return panels;
}

void Segmentations_Viewer::setSegmentations()
{
    segmentations->clear();
    _texture_Viewer->clearImageLabel();

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


