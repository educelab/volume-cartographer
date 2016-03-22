//
// Created by Seth Parker on 3/22/16.
//

#ifndef VC_MWINDOW_H
#define VC_MWINDOW_H

#include <iostream>

#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>

#include <QMainWindow>
#include <QMessageBox>
#include <QListWidget>
#include <QGraphicsView>
#include <QFileDialog>

#include "vc_defines.h"
#include "vc_datatypes.h"
#include "volumepkg.h"

#include "ui_mWindow.h"

namespace fs = boost::filesystem;

namespace Ui {
    class mWindow;
}

namespace volcart {
  class mWindow : public QMainWindow {

  Q_OBJECT

  public:
      mWindow();
      mWindow(QRect screenSize);
      ~mWindow( void );

  protected slots:
      void Open( void );
      void OnSegItemClicked( QListWidgetItem* item );
      //void OnTextureClick( void );

  private:
      ///// Load Data /////
      void OpenVolume( void );
      void CloseVolume( void );
      bool InitializeVolPkg( fs::path path );
      void InitializeSegList( void );

      ///// QT /////
      Ui::mWindow *ui;

      QListWidget    *_segList;
      QGraphicsView  *_textureViewer;
      QGraphicsView  *_sliceViewer;

      QAction  *_actionOpen;

      ///// Private data /////
      VolumePkg* _volpkg;
      std::string _activeSeg;
      fs::path _segPath;
      cv::Mat    _texture;
      cv::Mat    _perPixelMask;
      cv::Mat    _currentSlice;
      volcart::PerPixelMap _perPixelMap;
      cv::Vec2D _currentPoint;

      ///// State /////
      bool _viewChanged;

      ///// Helper functions /////
      cv::Vec3d lookup2Dto3D( int x, int y);
      cv::Mat   Convert8UC3( cv::Mat img );

      template < typename T >
      inline static void deleteNULL( T* &nPtr, bool nIsArray = false )
      {
        if ( nPtr != NULL ) {
          if ( nIsArray ) {
            delete []nPtr;
          } else {
            delete nPtr;
          }
          nPtr = NULL;
        }
      }
  };
}

#endif //VC_MWINDOW_H
