//
// Created by Seth Parker on 3/22/16.
//

#include <qapplication.h>
#include <QDesktopWidget>

#include "mWindow.h"

int main( int argc, char *argv[] )
{
  QApplication app( argc, argv );
  QRect rec = QApplication::desktop()->screenGeometry();// Instantiates a QRec "Rectangle" Object and gives it the dimensions of the screen display
  volcart::mWindow aWin(rec);
  aWin.show();
  return app.exec();

}
