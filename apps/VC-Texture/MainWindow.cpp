// MainWindow.cpp file for MainWindow Class , (Implements QMainWindow)
// Purpose: Create a Main Window for the GUI
// Developer: Michael Royal - mgro224@g.uky.edu
// October 12, 2015 - Spring Semester 2016
// Last Updated 09/26/2016 by: Michael Royal

// Copyright 2015 (Brent Seales: Volume Cartography Research)
// University of Kentucky VisCenter

#include <regex>

#include <boost/algorithm/string/case_conv.hpp>
#include <opencv2/imgcodecs.hpp>

#include "MainWindow.hpp"
#include "vc/core/io/OBJWriter.hpp"

// Volpkg version required by this app
static constexpr int VOLPKG_SUPPORTED_VERSION = 5;

namespace fs = boost::filesystem;

MainWindow::MainWindow(GlobalValues* globals)
{
    globals_ = globals;  // Enables access to Global Values Object

    setWindowTitle("VC Texture");  // Set Window Title

    // NOTE: Minimum Height and Width -------------------------
    // will be different on other display screens,
    // if Resolution is too small may cause distortion
    // of Buttons Visually when Program first Initiates

    // MIN DIMENSIONS
    window()->setMinimumHeight(globals_->getHeight() / 2);
    window()->setMinimumWidth(globals_->getWidth() / 2);
    // MAX DIMENSIONS
    window()->setMaximumHeight(globals_->getHeight());
    window()->setMaximumWidth(globals_->getWidth());

    // Create new TextureViewer Object (Left Side of GUI Display)
    auto textureViewer = new TextureViewer(globals);
    // Create new SegmentationsViewer Object (Right Side of GUI Display)
    auto segViewer = new SegmentationsViewer(globals, textureViewer);
    segViewer_ = segViewer;

    QHBoxLayout* mainLayout = new QHBoxLayout();
    mainLayout->addLayout(textureViewer->getLayout());
    mainLayout->addLayout(segViewer->getLayout());

    QWidget* w = new QWidget();
    w->setLayout(mainLayout);

    // Display Window
    setCentralWidget(w);

    setupActions();
    setupMenus();
}

// Gets the Folder Path of the Volume Package location, and initiates a VolPkg
void MainWindow::getFilePath()
{
    // If processing...
    if (globals_->getStatus() == ThreadStatus::Active) {
        QMessageBox::information(
            this, tr("Error Message"), "Please Wait While Texture Generates.");
        return;
    }

    // If there's something to save...
    if (globals_->getStatus() == ThreadStatus::Successful) {

        // Ask User to Save unsaved Data
        QMessageBox msgBox;
        msgBox.setText(
            "The texture image has not be saved. Do you want to save before "
            "closing?");
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
                globals_->setThreadStatus(ThreadStatus::Inactive);
                break;
            // Cancel was clicked
            case QMessageBox::Cancel:
                return;
            default:
                // should never be reached
                break;
        }
    }

    globals_->setThreadStatus(ThreadStatus::Inactive);
    resetGUI();
    auto dialogBox = new QFileDialog();
    fs::path filename = dialogBox->getExistingDirectory().toStdString();

    // If the user selected a Folder Path
    if (!filename.empty()) {

        // Checks the Folder Path for .volpkg extension
        std::regex volpkg(".*volpkg$");
        if (std::regex_match(filename.extension().string(), volpkg)) {
            try {
                // Sets Folder Path in Globals
                globals_->setVolgPkgPath(filename.c_str());
                globals_->loadVolPkg();

                // Check for Volume Package Version Number
                auto version = globals_->volPkg()->version();
                if (version != VOLPKG_SUPPORTED_VERSION) {
                    std::cerr << "VC::Error: Volume package is version "
                              << version
                              << " but this program requires a version "
                              << VOLPKG_SUPPORTED_VERSION << "." << std::endl;

                    QMessageBox::warning(
                        this, tr("ERROR"),
                        "Volume package is version " +
                            QString::number(version) +
                            " but this program requires version " +
                            QString::number(VOLPKG_SUPPORTED_VERSION) + ".");

                    // Reset Values
                    globals_->unloadVolPkg();
                    globals_->setVolgPkgPath("");
                    return;
                }

                // Gets Segmentations and assigns them to Globals
                globals_->loadSegIDs();
                // Initialize Segmentations in segmentations_Viewer
                segViewer_->setSegmentations();
                // Sets the name of the Volume Package to display on the GUI
                auto name = globals_->volPkg()->name();
                segViewer_->setVolPkgLabel(name.c_str());
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
    // If processing...
    if (globals_->getStatus() == ThreadStatus::Active) {
        QMessageBox::information(
            this, tr("Error Message"), "Please Wait While Texture Generates.");
        return;
    }

    // If A Volume Package is Loaded and there are Segmentations (continue)
    if (globals_->pkgLoaded() && globals_->volPkg()->hasSegmentations()) {
        // Checks to see if there are images
        if (globals_->getRendering().getTexture().hasImages()) {
            try {
                auto path =
                    globals_->getActiveSegmentation()->path() / "textured.obj";
                volcart::io::OBJWriter writer;
                writer.setPath(path.string());
                writer.setMesh(globals_->getRendering().getMesh());
                writer.setUVMap(globals_->getRendering().getTexture().uvMap());
                writer.setTexture(
                    globals_->getRendering().getTexture().image(0));
                writer.write();
                globals_->setThreadStatus(ThreadStatus::Inactive);
                QMessageBox::information(
                    this, tr("Error Message"), "Saved Successfully.");

            } catch (...) {
                QMessageBox::warning(
                    globals_->getWindow(), "Error",
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
    // If processing...
    if (globals_->getStatus() == ThreadStatus::Active) {
        QMessageBox::information(
            this, tr("Error Message"), "Please Wait While Texture Generates.");
        return;
    }

    // Return if no vopkg is loaded or if volpkg doesn't have segmentations
    if (!globals_->pkgLoaded() || !globals_->volPkg()->hasSegmentations()) {
        QMessageBox::warning(
            this, "Error",
            "Volume package not loaded/no segmentations in volume.");
        std::cerr << "vc::export::error: no volpkg loaded" << std::endl;
        return;
    }

    cv::Mat output;

    // Export the new texture first, otherwise the one already saved to disk
    if (globals_->getRendering().getTexture().hasImages()) {
        output = globals_->getRendering().getTexture().image(0);
    } else {
        auto path = globals_->getActiveSegmentation()->path() / "textured.png";
        output = cv::imread(path.string(), -1);
    }

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
        std::cerr << "vc::export::status: dialog cancelled." << std::endl;
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
        std::cerr << "vc::export::status: downsampled to 8U" << std::endl;
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

    std::cerr << "vc::export::status: export successful" << std::endl;
}

void MainWindow::setupActions()
{
    actionGetFilePath_ = new QAction("Open Volume...", this);
    connect(actionGetFilePath_, SIGNAL(triggered()), this, SLOT(getFilePath()));

    actionSave_ = new QAction("Save Texture", this);
    connect(actionSave_, SIGNAL(triggered()), this, SLOT(saveTexture()));

    actionExport_ = new QAction("Export Texture", this);
    connect(actionExport_, SIGNAL(triggered()), this, SLOT(exportTexture()));
}

void MainWindow::setupMenus()
{
    fileMenu_ = new QMenu(tr("&File"), this);
    globals_->setFileMenu(fileMenu_);

    fileMenu_->addAction(actionGetFilePath_);
    fileMenu_->addAction(actionSave_);
    fileMenu_->addAction(actionExport_);

    menuBar()->addMenu(fileMenu_);
}

// User cannot exit program while texture is still running.
void MainWindow::closeEvent(QCloseEvent* closing)
{
    if (globals_->getStatus() == ThreadStatus::Active) {
        QMessageBox::warning(
            this, "Error",
            "This application cannot be closed while a texture is being "
            "generated. Please wait until the texturing process is complete "
            "and try again.");
        closing->ignore();
        return;
    } else if (globals_->getStatus() == ThreadStatus::Successful) {

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
                break;
            // Cancel was clicked
            case QMessageBox::Cancel:
                closing->ignore();
                return;
            default:
                break;
        }
    }

    // Exit
    closing->accept();
}

void MainWindow::resetGUI()
{
    globals_->resetGUI();
    segViewer_->clearGUI();
    update();
}
