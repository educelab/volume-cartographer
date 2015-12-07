
#include "MyThread.h"

MyThread::MyThread(Global_Values *globals)
{
    _globals = globals;
    _globals->setStatus(0); // Status Running/Active
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
            newTexture = volcart::texturing::compositeTexture(mesh, *_globals->getVolPkg(), meshWidth, meshHeight, _radius, aFilterOption, aDirectionOption);

            _globals->setTexture(newTexture);

    }catch(...)
    {
        if(cloudProblem)
        {
            _globals->setStatus(-1);

        }else {
                _globals->setStatus(-2);
              }

    };

    if(_globals->getStatus()==0)
    {
        _globals->setStatus(1);
    }

    _globals->setProcessing(false);
}