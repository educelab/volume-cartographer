//----------------------------------------------------------------------------------------------------------------------------------------
// SegmentationsViewer.cpp file for SegmentationsViewer Class
// Purpose: Create SegmentationsViewer Class
// Developer: Michael Royal - mgro224@g.uky.edu
// October 12, 2015 - Spring Semester 2016
// Last Updated 11/13/2015 by: Michael Royal

// Copyright 2015 (Brent Seales: Volume Cartography Research)
// University of Kentucky VisCenter
//----------------------------------------------------------------------------------------------------------------------------------------

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "SegmentationsViewer.hpp"

#include "vc/texturing/CompositeTexture.hpp"

namespace vct = volcart::texturing;

SegmentationsViewer::SegmentationsViewer(
    GlobalValues* globals, TextureViewer* texture_Viewer)
{
    _globals = globals;
    _texture_Viewer = texture_Viewer;
    currentHighlightedIndex = 0;

    // RIGHT SIDE OF GUI
    //********************************************************************************************

    panels = new QVBoxLayout();

    // Volume_Package Section
    //----------------------------------------------------------
    volume_Package = new QLabel("Volume_Package");
    volume_Package->setMaximumWidth(400);
    segmentations = new QListWidget();
    segmentations->setMaximumSize(400, _globals->getHeight());

    panels->addWidget(volume_Package);
    panels->addWidget(segmentations);
    // End of Volume_Package Section
    //-----------------------------------------------------------

    // Parameters Section
    //-------------------------------------------------------------

    parameters = new QLabel("Parameters");
    radius = new QSpinBox();
    texture_Method = new QComboBox();
    texture_Filter = new QComboBox();
    sample_Direction = new QComboBox();
    generate = new QPushButton("Generate Texture");

    texture_Method->addItem("Intersection");
    texture_Method->addItem("Integral");
    texture_Method->addItem("Composite");

    texture_Filter->setEnabled(false);
    texture_Filter->addItem("Minimum");
    texture_Filter->addItem("Maximum");
    texture_Filter->addItem("Median");
    texture_Filter->addItem("Mean");
    texture_Filter->addItem("Median w/ Averaging");
    connect(
        texture_Method, QOverload<int>::of(&QComboBox::currentIndexChanged),
        [=](int v) { texture_Filter->setEnabled(v == 2); });

    sample_Direction->addItem("Omni");
    sample_Direction->addItem("Positive");
    sample_Direction->addItem("Negative");

    inputs = new QFormLayout();
    inputs->addRow("Radius: (Voxels)", radius);
    inputs->addRow("Texture Method:", texture_Method);
    inputs->addRow("Composite Filter:", texture_Filter);
    inputs->addRow("Sample Direction:", sample_Direction);

    user_input = new QVBoxLayout();
    user_input->addWidget(parameters);
    user_input->addLayout(inputs);
    user_input->addWidget(generate);

    panels->addLayout(user_input);

    // End of Parameters Section
    //-------------------------------------------------------------

    connect(
        segmentations, SIGNAL(itemClicked(QListWidgetItem*)), this,
        SLOT(itemClickedSlot()));
    connect(generate, SIGNAL(released()), this, SLOT(generateTextureImage()));

    // END OF RIGHT SIDE OF GUI
    //********************************************************************************************

}  // End of Default Constructor()

void SegmentationsViewer::itemClickedSlot()
{
    if (currentSegmentation != segmentations->currentItem()->text()) {

        // Check Status...
        if (_globals->getStatus() == ThreadStatus::Successful) {

            // Ask User to Save unsaved Data
            QMessageBox msgBox;
            msgBox.setWindowTitle("Discard changes?");
            msgBox.setText(tr(
                "Changes will be lost! Discard changes before continuing?\n"));
            msgBox.setStandardButtons(
                QMessageBox::Discard | QMessageBox::Cancel);
            msgBox.setDefaultButton(QMessageBox::Cancel);
            int option = msgBox.exec();

            switch (option) {
                case QMessageBox::Discard:
                    // Discard was clicked
                    _globals->setThreadStatus(ThreadStatus::Inactive);
                    break;
                case QMessageBox::Cancel:
                    // Cancel was clicked
                    segmentations->setCurrentRow(currentHighlightedIndex);
                    return;
                default:
                    // should never be reached
                    return;
            }
        } else {
            _globals->setThreadStatus(ThreadStatus::Inactive);
        }

        currentHighlightedIndex = segmentations->currentRow();
        currentSegmentation = segmentations->currentItem()->text();
        _globals->clearRendering();
        _texture_Viewer->clearImageLabel();

        QString s = segmentations->currentItem()->text();
        _globals->setActiveSegmentation(s.toStdString());

        auto path = _globals->getActiveSegmentation()->path() / "textured.png";
        cv::Mat texture = cv::imread(path.string(), -1);

        bool test = loadImage(texture);

        if (test) {
            _texture_Viewer->setImage();
        }
    }
}

void SegmentationsViewer::setVol_Package_Name(QString name)
{
    const QChar test = '/';
    const QString _name = name;
    QString file = _name;
    int index = -1;

    for (int i = 0; i < _name.length(); i++) {
        if (_name[i] == test) {
            index = i;
        }
    }

    std::string filename = _name.toStdString();

    if (index != -1) {
        filename = filename.substr(index + 1, filename.length());
    }

    const QString _filename = QString::fromStdString(filename);
    volume_Package->setText("Volume Package: " + _filename);
}

void SegmentationsViewer::generateTextureImage()
{
    if (!_globals->isVpkgInstantiated() ||
        _globals->getSegmentations().empty()) {
        QMessageBox::warning(
            _globals->getWindow(), "Error",
            "No Segmentation has been loaded, Please load Segmentation.");
        return;
    }
    // save current configuration
    auto flags = _globals->getWindow()->windowFlags();
    QSize size = _globals->getWindow()->frameSize();

    _globals->getWindow()->setWindowFlags(
        Qt::CustomizeWindowHint | Qt::WindowTitleHint |
        Qt::WindowMinimizeButtonHint | Qt::WindowMaximizeButtonHint);
    _globals->getWindow()->resize(size);
    _globals->getWindow()->show();

    _globals->setRadius(radius->text().toDouble());
    _globals->setTextureMethod(
        static_cast<GlobalValues::Method>(texture_Method->currentIndex()));
    _globals->setFilter(static_cast<vct::CompositeTexture::Filter>(
        texture_Filter->currentIndex()));
    _globals->setSampleDirection(sample_Direction->currentIndex());

    setEnabled(false);

    // enable the progress bar
    _texture_Viewer->progressActive(true);
    qApp->processEvents();

    // Create new thread and set for cleanup when complete
    processing = new MyThread(_globals);
    connect(processing, SIGNAL(finished()), processing, SLOT(deleteLater()));

    while (_globals->getStatus() == ThreadStatus::Active) {
        qApp->processEvents();
    }

    // Set Processing Status to Foced Close if Cancelled...
    if (_globals->getStatus() == ThreadStatus::ForcedClose) {
        processing->terminate();  // Clean up Threading
        processing->wait();
    }

    _texture_Viewer->progressActive(false);

    // If Processing successfully loaded an Image
    switch (_globals->getStatus()) {
        case ThreadStatus::Successful:
            if (loadImage(
                    _globals->getRendering().getTexture().image(0).clone())) {
                _texture_Viewer->setImage();
            } else {
                QMessageBox::warning(
                    _globals->getWindow(), "Error", "Failed to load image");
            }
            break;
        case ThreadStatus::CloudError:
            QMessageBox::warning(
                _globals->getWindow(), "Error",
                "Input point set is empty or has only one row");
            break;
        case ThreadStatus::Failed:
            QMessageBox::warning(
                _globals->getWindow(), "Error",
                "Failed to Generate Texture Image.");
            break;
        case ThreadStatus::ForcedClose:
            QMessageBox::warning(
                _globals->getWindow(), "Cancelled", "Successfully Cancelled.");
            break;
        default:
            break;
    }

    // Allow User to Use Buttons
    setEnabled(true);

    // restore
    _globals->getWindow()->setWindowFlags(flags);
    _globals->getWindow()->show();
}

bool SegmentationsViewer::loadImage(cv::Mat texture)
{
    try {
        if (texture.data == nullptr) {
            _texture_Viewer->clearImageLabel();
            QMessageBox::warning(
                _globals->getWindow(), "Error",
                "There is no Current Texture Image");
            return false;

        } else {
            // Convert to QPixMap and Display
            texture.convertTo(texture, CV_8U, 255.0 / 65535.0);
            cv::cvtColor(texture, texture, cv::COLOR_GRAY2RGB);

            QImage Image(
                texture.data, texture.cols, texture.rows, texture.step,
                QImage::Format_RGB888);
            newImage = Image;
            _globals->setQPixMapImage(newImage);
            return true;
        }

    } catch (...) {
        QMessageBox::warning(
            _globals->getWindow(), "Error", "Failed to Load Image Properly!");
    }

    return false;  // Should not reach here if successful
}

QVBoxLayout* SegmentationsViewer::getLayout() { return panels; }

void SegmentationsViewer::clearGUI()
{

    volume_Package->setText("");
    segmentations->clear();  // Clear the Segmentation List
    currentSegmentation = "";
    radius->setValue(0);
    texture_Method->setCurrentIndex(0);
    sample_Direction->setCurrentIndex(0);
    _texture_Viewer->clearGUI();  // Clear variables from _texture_Viewer
}

void SegmentationsViewer::setSegmentations()
{
    segmentations->clear();
    _texture_Viewer->clearImageLabel();

    std::vector<std::string> segments = _globals->getSegmentations();
    QString qstr;

    for (size_t i = 0; i < _globals->getSegmentations().size(); i++) {
        qstr = QString::fromStdString(segments[i]);
        segmentations->addItem(qstr);
    }

    if (_globals->getSegmentations().size() >
        0)  // Loads first Image to Screen by default
    {
        segmentations->setCurrentRow(0);
        currentHighlightedIndex = 0;
        currentSegmentation = "null";
        itemClickedSlot();
    }
}

void SegmentationsViewer::setEnabled(bool value)
{
    segmentations->setEnabled(value);
    radius->setEnabled(value);
    texture_Method->setEnabled(value);
    sample_Direction->setEnabled(value);
    generate->setEnabled(value);

    _globals->enableMenus(value);

    if (!value) {
        _texture_Viewer->clearLabel();
        _texture_Viewer->setEnabled(value);
    }
}
