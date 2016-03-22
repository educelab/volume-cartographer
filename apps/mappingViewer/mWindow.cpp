//
// Created by Seth Parker on 3/22/16.
//

#include "mWindow.h"

namespace volcart {

    ///// Constructors & Destructors /////

    // Constructor with QRect windowSize
    mWindow::mWindow(QRect screenSize) :
            _volpkg(NULL),
            ui(new Ui::mWindow)
    {
      ui->setupUi(this);

      int height = screenSize.height();
      int width = screenSize.width();

      //MIN DIMENSIONS
      window()->setMinimumHeight(height / 2);
      window()->setMinimumWidth(width / 2);
      //MAX DIMENSIONS
      window()->setMaximumHeight(height);
      window()->setMaximumWidth(width);

      // Widget stuff
      _segList = ui->segList;
      connect( _segList, SIGNAL( itemClicked( QListWidgetItem* ) ), this, SLOT( OnSegItemClicked( QListWidgetItem* ) ) );
      _textureViewer = ui->textureViewer;
      _sliceViewer = ui->sliceViewer;

      // Menu stuff
      connect( ui->actionOpen, SIGNAL( triggered() ), this, SLOT( Open() ) );

      update();
    };

    mWindow::~mWindow( void ) {
      deleteNULL(_volpkg);
      delete ui;
    };

    ///// Data IO /////
    // Ask for and attempt to load a volpkg
    void mWindow::OpenVolume( void ) {
      QString aVpkgPath = QString( "" );
      aVpkgPath = QFileDialog::getExistingDirectory( this,
                                                     tr( "Open Directory" ),
                                                     QDir::homePath(),
                                                     QFileDialog::ShowDirsOnly |
                                                     QFileDialog::DontResolveSymlinks );
      // Dialog box cancelled
      if ( aVpkgPath.length() == 0 ) {
        std::cerr << "VC::Message: Open volume package cancelled." << std::endl;
        return;
      }

      // Checks the Folder Path for .volpkg extension
      std::string extension = aVpkgPath.toStdString().substr( aVpkgPath.toStdString().length() - 7, aVpkgPath.toStdString().length() );
      if ( extension.compare(".volpkg") != 0 ) {
        QMessageBox::warning(this, tr("ERROR"), "The selected file is not of the correct type: \".volpkg\"");
        std::cerr << "VC::Error: Selected file: " << aVpkgPath.toStdString() << " is of the wrong type." << std::endl;
        _volpkg = NULL; // Is need for User Experience, clears screen.
        return;
      }

      // Open volume package
      if ( !InitializeVolPkg( aVpkgPath.toStdString() ) ) {
        return;
      }

      // Check version number
      if ( _volpkg->getVersion() < 2.0) {
        std::cerr << "VC::Error: Volume package is version " << _volpkg->getVersion() << " but this program requires a version >= 2.0." << std::endl;
        QMessageBox::warning( this, tr( "ERROR" ), "Volume package is version " + QString::number(_volpkg->getVersion()) + " but this program requires a version >= 2.0." );
        _volpkg = NULL;
        return;
      }

    }

    // Close this volume package
    void mWindow::CloseVolume() {
      _volpkg = NULL;
      _texture = cv::Mat();
      _perPixelMask = cv::Mat();
      _currentSlice = cv::Mat();
      _perPixelMap = volcart::PerPixelMap();
    }

    // Attempt to read volume package from disk
    bool mWindow::InitializeVolPkg( fs::path path )
    {
      deleteNULL( _volpkg );

      try {
        _volpkg = new VolumePkg( path.string() );
      } catch(...) {
        std::cerr << "mappingViewer::Error: Volume package failed to initialize." << std::endl;
      }

      if ( _volpkg == NULL ) {
        std::cerr << "VC::Error: Cannot open volume package at specified location: " << fs::canonical(path) << std::endl;
        QMessageBox::warning(this, "Error", "Volume package failed to load. Package might be corrupt.");
        return false;
      }

      return true;
    }

    // Fill the segmentation list
    void mWindow::InitializeSegList( void ) {
      _segList->clear();
      if (_volpkg != NULL) {
        // show the existing paths
        for (size_t i = 0; i < _volpkg->getSegmentations().size(); ++i) {
          _segList->addItem(new QListWidgetItem(QString(_volpkg->getSegmentations()[i].c_str())));
        }
      }
    }

    ///// Slots /////
    void mWindow::Open( void ) {
      CloseVolume();
      OpenVolume();
      InitializeSegList();
    }

    void mWindow::OnSegItemClicked( QListWidgetItem* item ) {
      if ( _activeSeg == item->text().toStdString() )
        return;

      _activeSeg = item->text().toStdString();
      _volpkg->setActiveSegmentation( _activeSeg );
      _segPath = fs::canonical(fs::path(_volpkg->getMeshPath()).parent_path());

      // Load the images and stuff
      _texture = cv::imread( _segPath.string() + "/textured.png" );
      _perPixelMask = cv::imread( _segPath.string() + "/PerPixelMask.png", CV_LOAD_IMAGE_GRAYSCALE );
      _perPixelMap.read( _segPath.string() + "/PerPixelMapping.yml.gz" );
      _currentSlice = Convert8UC3( _volpkg->volume().getSliceDataCopy(0) );

      _viewChanged = false;
    }

//    void mWindow::OnTextureClick(void) {
//      std::cerr << "callback" << std::endl;
//      // Filter for clicks
//      if ( event != cv::EVENT_LBUTTONDOWN )
//        return;
//
//      // Check the mask for to make sure we have a lookup
//      int maskVal = _perPixelMask.at< unsigned char >(y,x);
//      if ( maskVal == 0)
//        return;
//
//      // Get the 3D position
//      cv::Vec3d pos3D = lookup2Dto3D( y, x );
//
//      // Load the slice
//      _currentSlice = loadConverted( cvRound(pos3D(2)) );
//
//      // Draw a circle
//      cv::circle( _currentSlice, cvPoint( pos3D(0), pos3D(1) ), 3, cv::Scalar(0,0,255), -1 );
//
//      // Show the updated
//      cv::imshow("slice", _currentSlice);
//    };


    ///// Helper Functions /////
    cv::Vec3d mWindow::lookup2Dto3D(int y, int x) {
      return cv::Vec3d(_perPixelMap(y, x)(0), _perPixelMap(y, x)(1), _perPixelMap(y, x)(2));
    }

    cv::Mat mWindow::Convert8UC3( cv::Mat img ) {
      img /= 255.0;
      img.convertTo(img, CV_8UC3);
      cv::cvtColor(img, img, CV_GRAY2BGR);

      return img;
    };

}// namespace volcart