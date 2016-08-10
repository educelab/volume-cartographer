
#ifndef VC_MYTHREAD_H
#define VC_MYTHREAD_H

#include <QString>
#include <QThread>
#include <QLabel>

#include <vtkCleanPolyData.h>

#include "meshing/ACVD.h"
#include "texturing/AngleBasedFlattening.h"
#include "texturing/compositeTextureV2.h"
#include "common/util/meshMath.h"

#include "Global_Values.h"


class MyThread : public QThread
{

public:

    explicit MyThread(Global_Values *globals);
    void run();

private:

    Global_Values *_globals;

    static const uint16_t ABF_MIN_REQ_POINTS = 200;

};
#endif //VC_MYTHREAD_H
