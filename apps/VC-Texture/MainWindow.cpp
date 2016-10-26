//----------------------------------------------------------------------------------------------------------------------------------------
// MainWindow.cpp file for MainWindow Class , (Implements QMainWindow)
// Purpose: Create a Main Window for the GUI
// Developer: Michael Royal - mgro224@g.uky.edu
// October 12, 2015 - Spring Semester 2016
// Last Updated 09/26/2016 by: Michael Royal

// Copyright 2015 (Brent Seales: Volume Cartography Research)
// University of Kentucky VisCenter
//----------------------------------------------------------------------------------------------------------------------------------------

#include "MainWindow.h"
#include <boost/algorithm/string/case_conv.hpp>
#include "common/io/objWriter.h"

// Volpkg version required byt this app
static constexpr int VOLPKG_SUPPORTED_VERSION = 3;

namespace fs = boost::filesystem;

MainWindow::MainWindow(Global_Values* globals)
{
    _globals = globals;  // Enables access to Global Values Object

    setWindowTitle("VC Texture");  // Set Window Title

    // NOTE: Minimum Height and Width -------------------------
    // will be different on other display screens,
    // if Resolution is too small may cause distortion
    // of Buttons Visually when Program first Initiates
    //----------------------------------------------------------

    // MIN DIMENSIONS
    window()->setMinimumHeight(_globals->getHeight() / 2);
    window()->setMinimumWidth(_globals->getWidth() / 2);
    // MAX DIMENSIONS
    window()->setMaximumHeight(_globals->getHeight());
    window()->setMaximumWidth(_globals->getWidth());
    //---------------------------------------------------------

    // Create new Texture_Viewer Object (Left Side of GUI Display)
    Texture_Viewer* texture_Image = new Texture_Viewer(globals);
    // Create new Segmentations_Viewer Object (Right Side of GUI Display)
    Segmentations_Viewer* segmentations =
        new Segmentations_Viewer(globals, texture_Image);
    _segmentations_Viewer = segmentations;

    QHBoxLayout* mainLayout = new QHBoxLayout();
    mainLayout->addLayout(
        texture_Image->getLayout());  // THIS LAYOUT HOLDS THE WIDGETS FOR THE
                                      // OBJECT "Texture_Viewer" which Enables
                                      // the user to view images, zoom in, zoom
                                      // out, and reset the image.
    mainLayout->addLayout(
        segmentations->getLayout());  // THIS LAYOUT HOLDS THE WIDGETS FOR THE
                                      // OBJECT "Segmentations_Viewer" which
                                      // Enables the user to load segmentations,
                                      // and generate new texture images.

    QWidget* w = new QWidget();  // Creates the Primary Widget to display GUI
                                 // Functionality
    w->setLayout(
        mainLayout);  // w(the main window) gets assigned the mainLayout

    // Display Window
    //------------------------------
    setCentralWidget(
        w);  // w is a wrapper widget for all of the widgets in the main window.

    create_Actions();  // Creates the Actions for the Menu Bar & Sub-Menus
    create_Menus();    // Creates the Menus and adds them to the Menu Bar
}

void MainWindow::getFilePath()  // Gets the Folder Path of the Volume Package
                                // location, and initiates a Volume Package.
{
    // Check Status...
    if (_globals->getStatus() == ThreadStatus::Successful) {

        // Ask User to Save unsaved Data
        QMessageBox msgBox;
        msgBox.setText(
            "A new texture image was generated, do you want to save it?");
        msgBox.setStandardButtons(
            QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
        int option = msgBox.exec();

        switch (option) {

            // Save was clicked
            case QMessageBox::Save:
                saveTexture();
                break;

            // Discard was clicked
            case QMessageBox::Discard:
                _globals->setThreadStatus(ThreadStatus::Inactive);
                break;

            // Cancel was clicked
            case QMessageBox::Cancel:
                return;

            default:
                // should never be reached
                break;
        }
    } else {
        _globals->setThreadStatus(ThreadStatus::Inactive);
    }

    QFileDialog* dialogBox = new QFileDialog();
    QString filename = dialogBox->getExistingDirectory();
    std::string file_Name = filename.toStdString();

    if (!filename.isEmpty())  // If the user selected a Folder Path
    {

        if ((file_Name.substr(file_Name.length() - 7, file_Name.length()))
                .compare(".volpkg") ==
            0)  // Checks the Folder Path for .volpkg extension
        {
            try {
                _globals->setPath(filename);  // Sets Folder Path in Globals
                _globals
                    ->createVolumePackage();  // Creates a Volume Package Object

                // Check for Volume Package Version Number
                if (_globals->getVolPkg()->getVersion() !=
                    VOLPKG_SUPPORTED_VERSION) {

                    std::cerr << "VC::Error: Volume package is version "
                              << _globals->getVolPkg()->getVersion()
                              << " but this program requires a version "
                              << std::to_string(VOLPKG_SUPPORTED_VERSION) << "."
                              << std::endl;

                    QMessageBox::warning(
                        this, tr("ERROR"),
                        "Volume package is version " +
                            QString::number(
                                _globals->getVolPkg()->getVersion()) +
                            " but this program requires version " +
                            QString::number(VOLPKG_SUPPORTED_VERSION) + ".");

                    // Reset Values
                    _globals->clearVolumePackage();  // Reset Volume Package
                    _globals->setPath("");           // Clear filename path
                    return;
                }

                // Gets Segmentations and assigns them to "segmentations" in
                // Globals
                _globals->getMySegmentations();
                // Initialize Segmentations in segmentations_Viewer
                _segmentations_Viewer->setSegmentations();
                // Sets the name of the Volume Package to display on the GUI
                _segmentations_Viewer->setVol_Package_Name(filename);
            } catch (...) {
                QMessageBox::warning(
                    this, tr("Error Message"), "Error Opening File.");
            };
        } else {
            QMessageBox::warning(this, tr("Error Message"), "Invalid File.");
        }
    }
}

// Overrides the Current Texture Image in the Segmentation's Folder with the
// newly Generated Texture Image.
void MainWindow::saveTexture()
{
    // If A Volume Package is Loaded and there are Segmentations (continue)
    if (_globals->isVPKG_Intantiated() &&
        _globals->getVolPkg()->getSegmentations().size() != 0) {
        if (_globals->getRendering()
                .getTexture()
                .hasImages())  // Checks to see if there are images
        {
            try {
                fs::path path =
                    _globals->getVolPkg()->getActiveSegPath() / "textured.obj";
                volcart::io::objWriter mesh_writer;
                mesh_writer.setPath(path.string());
                mesh_writer.setRendering(_globals->getRendering());
                mesh_writer.write();
                _globals->setThreadStatus(ThreadStatus::Inactive);
                QMessageBox::information(
                    this, tr("Error Message"), "Saved Successfully.");

            } catch (...) {
                QMessageBox::warning(
                    _globals->getWindow(), "Error",
                    "Failed to Save Texture Image Properly!");
            }

        } else
            QMessageBox::information(
                this, tr("Error Message"),
                "Please Generate a New Texture Image.");

    } else
        QMessageBox::warning(
            this, tr("Error Message"), "There is no Texture Image to Save!");
}

// Exports the Image as .tif, .tiff, .png, .jpg, and .jpeg
void MainWindow::exportTexture()
{

    // Return if no volume package is loaded or if volpkg doesn't have
    // segmentations
    if (!_globals->isVPKG_Intantiated() ||
        !_globals->getVolPkg()->getSegmentations().size()) {
        QMessageBox::warning(
            this, "Error",
            "Volume package not loaded/no segmentations in volume.");
        std::cerr << "vc::export::error: no volpkg loaded" << std::endl;
        return;
    }

    cv::Mat output;

    // Export the generated texture first, otherwise the one already saved to
    // disk
    if (_globals->getRendering().getTexture().hasImages())
        output = _globals->getRendering().getTexture().getImage(0);
    else
        output = _globals->getVolPkg()->getTextureData();

    // Return if no image to export
    if (!output.data) {
        QMessageBox::warning(
            this, "Error",
            "No image to export. Please load a different segmentation or "
            "generate a new texture.");
        std::cerr << "vc::export::error: no image data to export" << std::endl;
        return;
    }

    // Get the output path
    fs::path outputPath;
    outputPath = QFileDialog::getSaveFileName(
                     this, tr("Export Texture Image"), "",
                     tr("Images (*.png *jpg *jpeg *tif *tiff)"))
                     .toStdString();

    // If no path provided/dialog cancelled
    if (outputPath.empty()) {
        std::cerr << "vc::export::_status: dialog cancelled." << std::endl;
        return;
    }

    ///// Deal with edge cases /////
    // Default to png if no extension provided
    if (outputPath.extension().empty())
        outputPath = outputPath.string() + ".png";

    // For convenience
    std::string extension(
        boost::to_upper_copy<std::string>(outputPath.extension().string()));

    // Check for approved format
    std::vector<std::string> approvedExtensions{".PNG", ".JPG", ".JPEG", ".TIF",
                                                ".TIFF"};
    auto it = std::find(
        approvedExtensions.begin(), approvedExtensions.end(), extension);
    if (it == approvedExtensions.end()) {
        QMessageBox::warning(
            this, "Error",
            "Unknown file format for export. Please use .png, .jpg, or .tif.");
        std::cerr << "vc::export::error: unknown output format: " << extension
                  << std::endl;
        return;
    }

    // Convert to 8U if JPG
    if (extension == ".JPG" || extension == ".JPEG") {
        output = output.clone();
        output.convertTo(output, CV_8U, 255.0 / 65535.0);
        std::cerr << "vc::export::_status: downsampled to 8U" << std::endl;
    }

    // Write the image
    try {
        cv::imwrite(outputPath.string(), output);
    } catch (std::runtime_error& ex) {
        QMessageBox::warning(this, "Error", "Error writing file.");
        std::cerr << "vc::export::error: exception writing image: " << ex.what()
                  << std::endl;
        return;
    }

    std::cerr << "vc::export::_status: export successful" << std::endl;
}

void MainWindow::create_Actions()
{
    actionGetFilePath = new QAction("Open Volume...", this);
    connect(actionGetFilePath, SIGNAL(triggered()), this, SLOT(getFilePath()));

    actionSave = new QAction("Save Texture", this);
    connect(actionSave, SIGNAL(triggered()), this, SLOT(saveTexture()));

    actionExport = new QAction("Export Texture", this);
    connect(actionExport, SIGNAL(triggered()), this, SLOT(exportTexture()));
}

void MainWindow::create_Menus()
{
    fileMenu = new QMenu(tr("&File"), this);
    _globals->setFileMenu(fileMenu);

    fileMenu->addAction(actionGetFilePath);
    fileMenu->addAction(actionSave);
    fileMenu->addAction(actionExport);

    menuBar()->addMenu(fileMenu);
}

// User cannot exit program while texture is still running.
void MainWindow::closeEvent(QCloseEvent* closing)
{
    if (_globals->getStatus() == ThreadStatus::Active) {
        QMessageBox::warning(
            this, "Error",
            "This application cannot be closed while a texture is being "
            "generated. Please wait until the texturing process is complete "
            "and try again.");
        closing->ignore();
    } else if (_globals->getStatus() == ThreadStatus::Successful) {

        // Ask User to Save unsaved Data
        QMessageBox msgBox;
        msgBox.setText(
            "A new texture image was generated, do you want to save it before "
            "quitting?");
        msgBox.setStandardButtons(
            QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel);
        int option = msgBox.exec();

        switch (option) {
            // Save was clicked
            case QMessageBox::Save:
                saveTexture();
                break;

            // Discard was clicked
            case QMessageBox::Discard:
                // Reset ThreadStatus
                _globals->setThreadStatus(ThreadStatus::Inactive);
                break;

            // Cancel was clicked
            case QMessageBox::Cancel:
                closing->ignore();
                return;

            default:
                // should never be reached
                break;
        }

        // Exit the program
        closing->accept();

    } else {
        closing->accept();
    }
}
