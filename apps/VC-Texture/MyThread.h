
#ifndef VC_MYTHREAD_H
#define VC_MYTHREAD_H

#include <QString>
#include <QThread>
#include <QLabel>

#include <vtkCleanPolyData.h>

#include "ACVD.h"
#include "AngleBasedFlattening.h"
#include "compositeTextureV2.h"
#include "meshMath.h"

#include "Global_Values.h"


class MyThread : public QThread
{

public:

    explicit MyThread(Global_Values *globals);
    void run();

private:

    Global_Values *_globals;

};
#endif //VC_MYTHREAD_H
