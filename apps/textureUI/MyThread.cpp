
#include "MyThread.h"
#include <iostream>

MyThread::MyThread(QString s, QLabel *label)
{
    name = s;
    _label = label;
}

void MyThread::run()
{
    ;

    //Testing
    while(true)
    {
        sleep(1);
        _label->setText("Loading.");
        sleep(1);
        _label->setText("Loading..");
        sleep(1);
        _label->setText("Loading...");

        std::cout<<"Hello World";
    }
}