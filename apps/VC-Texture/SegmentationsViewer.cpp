// SegmentationsViewer.cpp file for SegmentationsViewer Class
// Purpose: Create SegmentationsViewer Class
// Developer: Michael Royal - mgro224@g.uky.edu
// October 12, 2015 - Spring Semester 2016
// Last Updated 11/13/2015 by: Michael Royal

// Copyright 2015 (Brent Seales: Volume Cartography Research)
// University of Kentucky VisCenter

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "SegmentationsViewer.hpp"

#include "vc/texturing/CompositeTexture.hpp"

namespace vct = volcart::texturing;

SegmentationsViewer::SegmentationsViewer(
    GlobalValues* globals, TextureViewer* texture_Viewer)
{
    globals_ = globals;
    textureViewer_ = texture_Viewer;
    activeSegIndex_ = 0;

    // RIGHT SIDE OF GUI
    //********************************************************************************************

    panels_ = new QVBoxLayout();

    // Volume_Package Section
    volPkgLabel_ = new QLabel("Volume_Package");
    volPkgLabel_->setMaximumWidth(400);
    segList_ = new QListWidget();
    segList_->setMaximumSize(400, globals_->getHeight());

    panels_->addWidget(volPkgLabel_);
    panels_->addWidget(segList_);
    // End of Volume_Package Section

    // Parameters Section

    paramsLabel_ = new QLabel("Parameters");
    radiusSelect_ = new QSpinBox();
    methodSelect_ = new QComboBox();
    filterSelect_ = new QComboBox();
    directionSelect_ = new QComboBox();
    startGen_ = new QPushButton("Generate Texture");

    methodSelect_->addItem("Intersection");
    methodSelect_->addItem("Integral");
    methodSelect_->addItem("Composite");

    filterSelect_->setEnabled(false);
    filterSelect_->addItem("Minimum");
    filterSelect_->addItem("Maximum");
    filterSelect_->addItem("Median");
    filterSelect_->addItem("Mean");
    filterSelect_->addItem("Median w/ Averaging");
    connect(
        methodSelect_, QOverload<int>::of(&QComboBox::currentIndexChanged),
        [=](int v) { filterSelect_->setEnabled(v == 2); });

    directionSelect_->addItem("Omni");
    directionSelect_->addItem("Positive");
    directionSelect_->addItem("Negative");

    paramsContainer_ = new QFormLayout();
    paramsContainer_->addRow("Radius: (Voxels)", radiusSelect_);
    paramsContainer_->addRow("Texture Method:", methodSelect_);
    paramsContainer_->addRow("Composite Filter:", filterSelect_);
    paramsContainer_->addRow("Sample Direction:", directionSelect_);

    paramsSubcontainer_ = new QVBoxLayout();
    paramsSubcontainer_->addWidget(paramsLabel_);
    paramsSubcontainer_->addLayout(paramsContainer_);
    paramsSubcontainer_->addWidget(startGen_);

    panels_->addLayout(paramsSubcontainer_);

    // End of Parameters Section

    connect(
        segList_, SIGNAL(itemClicked(QListWidgetItem*)), this,
        SLOT(itemClicked()));
    connect(startGen_, SIGNAL(released()), this, SLOT(generateTexture()));

    // END OF RIGHT SIDE OF GUI
    //********************************************************************************************

}  // End of Default Constructor()

void SegmentationsViewer::itemClicked()
{
    if (activeSegID_ != segList_->currentItem()->text()) {

        // Check Status...
        if (globals_->getStatus() == ThreadStatus::Successful) {

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
                    globals_->setThreadStatus(ThreadStatus::Inactive);
                    break;
                case QMessageBox::Cancel:
                    // Cancel was clicked
                    segList_->setCurrentRow(activeSegIndex_);
                    return;
                default:
                    // should never be reached
                    return;
            }
        } else {
            globals_->setThreadStatus(ThreadStatus::Inactive);
        }

        activeSegIndex_ = segList_->currentRow();
        activeSegID_ = segList_->currentItem()->text();
        globals_->clearRendering();
        textureViewer_->clearImageArea();

        QString s = segList_->currentItem()->text();
        globals_->setActiveSeg(s.toStdString());

        auto path = globals_->getActiveSegmentation()->path() / "textured.png";
        cv::Mat texture = cv::imread(path.string(), -1);

        bool test = loadImage(texture);

        if (test) {
            textureViewer_->loadImageFromGlobals();
        }
    }
}

void SegmentationsViewer::setVolPkgLabel(QString name)
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
    volPkgLabel_->setText("Volume Package: " + _filename);
}

void SegmentationsViewer::generateTexture()
{
    if (!globals_->pkgLoaded() || globals_->getSegIDs().empty()) {
        QMessageBox::warning(
            globals_->getWindow(), "Error",
            "No Segmentation has been loaded, Please load Segmentation.");
        return;
    }
    // save current configuration
    auto flags = globals_->getWindow()->windowFlags();
    QSize size = globals_->getWindow()->frameSize();

    globals_->getWindow()->setWindowFlags(
        Qt::CustomizeWindowHint | Qt::WindowTitleHint |
        Qt::WindowMinimizeButtonHint | Qt::WindowMaximizeButtonHint);
    globals_->getWindow()->resize(size);
    globals_->getWindow()->show();

    globals_->setRadius(radiusSelect_->text().toDouble());
    globals_->setTextureMethod(
        static_cast<GlobalValues::Method>(methodSelect_->currentIndex()));
    globals_->setFilter(static_cast<vct::CompositeTexture::Filter>(
        filterSelect_->currentIndex()));
    globals_->setSampleDirection(directionSelect_->currentIndex());

    setEnabled(false);

    // enable the progress bar
    textureViewer_->setProgressActive(true);
    qApp->processEvents();

    // Create new thread and set for cleanup when complete
    procThread_ = new MyThread(globals_);
    connect(procThread_, SIGNAL(finished()), procThread_, SLOT(deleteLater()));

    while (globals_->getStatus() == ThreadStatus::Active) {
        qApp->processEvents();
    }

    // Set Processing Status to Foced Close if Cancelled...
    if (globals_->getStatus() == ThreadStatus::ForcedClose) {
        procThread_->terminate();  // Clean up Threading
        procThread_->wait();
    }

    textureViewer_->setProgressActive(false);

    // If Processing successfully loaded an Image
    switch (globals_->getStatus()) {
        case ThreadStatus::Successful:
            if (loadImage(
                    globals_->getRendering().getTexture().at(0).clone())) {
                textureViewer_->loadImageFromGlobals();
            } else {
                QMessageBox::warning(
                    globals_->getWindow(), "Error", "Failed to load image");
            }
            break;
        case ThreadStatus::CloudError:
            QMessageBox::warning(
                globals_->getWindow(), "Error",
                "Cannot generate texture! Input point set is empty or has only "
                "one row");
            break;
        case ThreadStatus::Failed:
            QMessageBox::warning(
                globals_->getWindow(), "Error",
                "Failed to Generate Texture Image.");
            break;
        case ThreadStatus::ForcedClose:
            QMessageBox::warning(
                globals_->getWindow(), "Cancelled", "Successfully Cancelled.");
            break;
        default:
            break;
    }

    // Allow User to Use Buttons
    setEnabled(true);

    // restore
    globals_->getWindow()->setWindowFlags(flags);
    globals_->getWindow()->show();
}

bool SegmentationsViewer::loadImage(cv::Mat texture)
{
    try {
        if (texture.data == nullptr) {
            textureViewer_->clearImageArea();
            QMessageBox::warning(
                globals_->getWindow(), "Error",
                "There is no Current Texture Image");
            return false;

        } else {
            // Convert to QPixMap and Display
            texture.convertTo(texture, CV_8U, 255.0 / 65535.0);
            cv::cvtColor(texture, texture, cv::COLOR_GRAY2RGB);

            QImage Image(
                texture.data, texture.cols, texture.rows, texture.step,
                QImage::Format_RGB888);
            displayImage_ = Image;
            globals_->setQPixMapImage(displayImage_);
            return true;
        }

    } catch (...) {
        QMessageBox::warning(
            globals_->getWindow(), "Error", "Failed to Load Image Properly!");
    }

    return false;  // Should not reach here if successful
}

QVBoxLayout* SegmentationsViewer::getLayout() { return panels_; }

void SegmentationsViewer::clearGUI()
{

    volPkgLabel_->setText("");
    segList_->clear();  // Clear the Segmentation List
    activeSegID_ = "";
    radiusSelect_->setValue(0);
    methodSelect_->setCurrentIndex(0);
    directionSelect_->setCurrentIndex(0);
    textureViewer_->clearGUI();  // Clear variables from textureViewer_
}

void SegmentationsViewer::setSegmentations()
{
    segList_->clear();
    textureViewer_->clearImageArea();

    std::vector<std::string> segments = globals_->getSegIDs();
    QString qstr;

    for (size_t i = 0; i < globals_->getSegIDs().size(); i++) {
        qstr = QString::fromStdString(segments[i]);
        segList_->addItem(qstr);
    }

    // Loads first Image to Screen by default
    if (globals_->getSegIDs().empty()) {
        segList_->setCurrentRow(0);
        activeSegIndex_ = 0;
        activeSegID_ = "null";
        itemClicked();
    }
}

void SegmentationsViewer::setEnabled(bool value)
{
    segList_->setEnabled(value);
    radiusSelect_->setEnabled(value);
    methodSelect_->setEnabled(value);
    directionSelect_->setEnabled(value);
    startGen_->setEnabled(value);

    globals_->enableMenus(value);

    if (!value) {
        textureViewer_->clearLabel();
        textureViewer_->setEnabled(value);
    }
}
