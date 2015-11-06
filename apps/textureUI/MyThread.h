
#ifndef VC_MYTHREAD_H
#define VC_MYTHREAD_H

#include <QString>
#include <QThread>
#include <QLabel>


class MyThread : public QThread
{

public:

    explicit MyThread(QString s, QLabel *label);
    void run();

private:
    QString name;
    QLabel *_label;

};



#endif //VC_MYTHREAD_H
