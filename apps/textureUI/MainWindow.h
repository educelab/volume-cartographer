


#include <QApplication>
#include <QtGui>
#include <QObject>
#include <QMainWindow>
#include <QMenu>
#include <QAction>
#include <QMenuBar>
#include "Global_Values.h"


class MainWindow : public QMainWindow
{
    // NOTICE THIS MACRO
    Q_OBJECT
    //

public:
    MainWindow(Global_Values *globals);

public slots:
    void getFilePath();
    void showNotes();

private:
    void create_Actions();
    void create_Menus();

    QMenuBar* menu_Bar;
    QMenu* fileMenu;
    QMenu* optionsMenu;
    QAction* actionGetFilePath;
    QAction* actionNotes;

    Global_Values *_globals;
};