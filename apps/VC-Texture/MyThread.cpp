
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

        vtkPolyData* vtkMesh = vtkPolyData::New();
        volcart::meshing::itk2vtk(mesh, vtkMesh);

        // Decimate using ACVD
        vtkPolyData* acvdMesh = vtkPolyData::New();
        volcart::meshing::ACVD(vtkMesh, acvdMesh, 2000, 0, 0, 200);

        // Smooth points
        vtkSmartPointer<vtkSmoothPolyDataFilter> smoothFilter = vtkSmartPointer<vtkSmoothPolyDataFilter>::New();
        smoothFilter->SetInputData( acvdMesh );
        smoothFilter->SetNumberOfIterations(50);
        smoothFilter->SetRelaxationFactor(0.05);
        smoothFilter->FeatureEdgeSmoothingOn();
        smoothFilter->BoundarySmoothingOff();
        smoothFilter->Update();

        // Merge Duplicates
        // Note: This merging has to be the last in the process chain for some really weird reason. - SP
        vtkSmartPointer<vtkCleanPolyData> Cleaner = vtkCleanPolyData::New();
        Cleaner->SetInputConnection( smoothFilter->GetOutputPort() );
        Cleaner->ToleranceIsAbsoluteOn();
        Cleaner->Update();

        // Convert back to ITK mesh
        VC_MeshType::Pointer outputMesh = VC_MeshType::New();
        volcart::meshing::vtk2itk( Cleaner->GetOutput(), outputMesh);

        // Compute parameterization
        volcart::texturing::lscm lscm( outputMesh );
        lscm.compute();

        // Get uv map
        volcart::UVMap uvMap = lscm.getUVMap();
        int width = std::ceil( uvMap.ratio().width );
        int height = std::ceil( (double) width / uvMap.ratio().aspect );

        volcart::texturing::compositeTextureV2 result(outputMesh, *_globals->getVolPkg(), uvMap, _radius, width, height, aFilterOption, aDirectionOption);

        // Setup rendering
        volcart::Rendering rendering;
        rendering.setTexture( result.texture() );
        rendering.setMesh( outputMesh );

        _globals->setRendering( rendering );

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