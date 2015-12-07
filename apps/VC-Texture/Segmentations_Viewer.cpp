//----------------------------------------------------------------------------------------------------------------------------------------
// Segmentations_Viewer.cpp file for Segmentations_Viewer Class
// Purpose: Create Segmentations_Viewer Class
// Developer: Michael Royal - mgro224@g.uky.edu
// October 12, 2015 - Spring Semester 2016
// Last Updated 11/13/2015 by: Michael Royal

// Copy Right ©2015 (Brent Seales: Volume Cartography Research) - University of Kentucky Center for Visualization and Virtualization
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
    if(currentSegmentation != segmentations->currentItem()->text())
    {
        currentSegmentation = segmentations->currentItem()->text();
        _globals->clearTexture();
        _texture_Viewer->clearImageLabel();

        QString s = segmentations->currentItem()->text();// Gets a QString for the Current Item Selected
        _globals->getVolPkg()->setActiveSegmentation(s.toStdString());// Sets the active Segmentation

        cv::Mat texture = _globals->getVolPkg()->getTextureData().clone();

        bool test = loadImage(texture);

        if(test)
        {
            _texture_Viewer->setImage();
        }
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
    if(_globals->isVPKG_Intantiated() && _globals->getSegmentations().size()>0)
    {
        auto flags = _globals->getWindow()->windowFlags();//save current configuration
        QSize size = _globals->getWindow()->frameSize();

        _globals->getWindow()->setWindowFlags ( Qt::CustomizeWindowHint | Qt::WindowTitleHint | Qt::WindowMinimizeButtonHint | Qt::WindowMaximizeButtonHint);
        _globals->getWindow()->resize(size);// Resizes
        _globals->getWindow()->show();// Show Screen

        _globals->setRadius(radius->text().toDouble());
        _globals->setTextureMethod(texture_Method->currentIndex());
        _globals->setSampleDirection(sample_Direction->currentIndex());

        setEnabled(false);

        _texture_Viewer->progressActive(true);// Show Progress Loading Bar
        qApp->processEvents();//Updates GUI Window

        processing = new MyThread(_globals);// Creates new thread
        connect(processing, SIGNAL(finished()), processing, SLOT(deleteLater()));// Deletes Thread After Completion (Clean-Up)

        while(_globals->getProcessing())
        {
            qApp->processEvents();//Updates GUI Window
        }

        // Set Processing Status to -999 if Cancelled...
        if(_globals->getForcedClose())
        {
            _globals->setStatus(-999);
            processing->terminate();// Clean up Threading
            processing->wait();
        }

        _texture_Viewer->progressActive(false);// Hide Progress Loading Bar

        bool test = false;

        if(_globals->getStatus()==1)
        {
            test = loadImage(_globals->getTexture().getImage(0).clone());

            if (test)
            {
                _texture_Viewer->setImage();
            }
        }

        if(_globals->getStatus()==1 && test)// If Processing successfully loaded an Image
            {
                QMessageBox::warning(_globals->getWindow(), "Warning", "The Generated Texture Image is not Saved, if you wish to save it, please select \"File\" -> \"Save Texture\".");

            }else if(_globals->getStatus()==-1)
                    {
                        QMessageBox::warning(_globals->getWindow(),"Error", "Failed to Generate Texture Image [cloud.ply] error.");

                    }else if(_globals->getStatus()==-2)
                            {
                                QMessageBox::warning(_globals->getWindow(),"Error", "Failed to Generate Texture Image.");

                            }else if(_globals->getStatus()==-999)
                                    {
                                        QMessageBox::warning(_globals->getWindow(),"Cancelled", "Successfully Cancelled.");
                                    }

        setEnabled(true);// Allow User to Use Buttons
        _globals->getWindow()->setWindowFlags(flags);//restore
        _globals->getWindow()->show();

    }else QMessageBox::warning(_globals->getWindow(),"Error", "No Segmentation has been loaded, Please load Segmentation.");

}// End of generateTextureImage()

bool Segmentations_Viewer::loadImage(cv::Mat texture)
{
    try
    {
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
                QMessageBox::warning(_globals->getWindow(), "Error", "Failed to Load Image Properly!");
        }

    return false; // Should not reach here if successful

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
        currentSegmentation = "null";
        itemClickedSlot();
    }
}

void Segmentations_Viewer::setEnabled(bool value)
{
    segmentations->setEnabled(value);
    radius->setEnabled(value);
    texture_Method->setEnabled(value);
    sample_Direction->setEnabled(value);
    generate->setEnabled(value);

    _globals->enableMenus(value);

if(!value)
    {
        _texture_Viewer->clearLabel();
        _texture_Viewer->setEnabled(value);
    }

}