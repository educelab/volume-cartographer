
#include "MyThread.h"

MyThread::MyThread(Global_Values *globals)
{
    _globals = globals;
    _status = 0; // Status Running/Active
    _globals->setProcessing(true);
    _globals->setForcedClose(false);
    this->start();
}

void MyThread::run()
{

    bool cloudProblem = false;

    try {
            double _radius = _globals->getRadius();
            int meshWidth = -1;
            int meshHeight = -1;

            std::string meshName = _globals->getVolPkg()->getMeshPath();

            VC_Composite_Option aFilterOption = (VC_Composite_Option) _globals->getTextureMethod();
            VC_Direction_Option aDirectionOption = (VC_Direction_Option) _globals->getSampleDirection();

            // declare pointer to new Mesh object
            VC_MeshType::Pointer mesh = VC_MeshType::New();

            // try to convert the ply to an ITK mesh
            if (!volcart::io::ply2itkmesh(meshName, mesh, meshWidth, meshHeight))
            {
                cloudProblem = true;
                throw(__EXCEPTIONS);// Error
            };

            volcart::Texture newTexture;
            newTexture = volcart::texturing::compositeTexture(mesh, *_globals->getVolPkg(), meshWidth, meshHeight, _radius, aFilterOption, aDirectionOption);// Save to Globals

            _globals->setTexture(newTexture);

            // Display this. This is a 16-bit, single channel image.
            cv::Mat texture = newTexture.getImage(0).clone();// Use to get Image

    }catch(...)
    {
        if(cloudProblem)
        {
            _status = -1;

        }else {
                _status = -2;
              }

    };

    if(_status==0)
    {
        _status = 1;
    }

    _globals->setProcessing(false);
}

void MyThread::setStatus(int status)
{
    _status = status;
}

int MyThread::getStatus()
{
    return _status;
}