
#include "MyThread.h"
#include "common/io/objWriter.h"
#include "common/io/ply2itk.h"

namespace fs = boost::filesystem;

MyThread::MyThread(Global_Values* globals)
{
    _globals = globals;
    _globals->setStatus(0);  // Status Running/Active
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

        fs::path meshName = _globals->getVolPkg()->getMeshPath();

        volcart::CompositeOption aFilterOption =
            (volcart::CompositeOption)_globals->getTextureMethod();
        volcart::DirectionOption aDirectionOption =
            (volcart::DirectionOption)_globals->getSampleDirection();

        // declare pointer to new Mesh object
        volcart::MeshType::Pointer mesh = volcart::MeshType::New();

        // try to convert the ply to an ITK mesh
        if (!volcart::io::ply2itkmesh(meshName, mesh, meshWidth, meshHeight)) {
            cloudProblem = true;
            throw(__EXCEPTIONS);  // Error
        };

        // Calculate sampling density
        double voxelsize = _globals->getVolPkg()->getVoxelSize();
        double sa = volcart::meshMath::SurfaceArea(mesh) *
                    (voxelsize * voxelsize) *
                    (0.001 * 0.001);  // convert vx^2 -> mm^2;
        double densityFactor = 50;
        uint16_t numberOfVertices = std::round(densityFactor * sa);
        numberOfVertices = (numberOfVertices < CLEANER_MIN_REQ_POINTS)
                               ? CLEANER_MIN_REQ_POINTS
                               : numberOfVertices;

        // Convert to polydata
        auto vtkMesh = vtkSmartPointer<vtkPolyData>::New();
        volcart::meshing::itk2vtk(mesh, vtkMesh);

        // Decimate using ACVD
        std::cout << "Resampling mesh..." << std::endl;
        auto acvdMesh = vtkSmartPointer<vtkPolyData>::New();
        volcart::meshing::ACVD(vtkMesh, acvdMesh, numberOfVertices);

        // Merge Duplicates
        // Note: This merging has to be the last in the process chain for some
        // really weird reason. - SP
        auto Cleaner = vtkSmartPointer<vtkCleanPolyData>::New();
        Cleaner->SetInputData(acvdMesh);
        Cleaner->Update();

        volcart::MeshType::Pointer itkACVD = volcart::MeshType::New();
        volcart::meshing::vtk2itk(Cleaner->GetOutput(), itkACVD);

        // ABF flattening
        std::cout << "Computing parameterization..." << std::endl;
        volcart::texturing::AngleBasedFlattening abf(itkACVD);
        // abf.setABFMaxIterations(5);
        abf.compute();

        // Get uv map
        volcart::UVMap uvMap = abf.getUVMap();
        int width = std::ceil(uvMap.ratio().width);
        int height = std::ceil((double)width / uvMap.ratio().aspect);

        volcart::texturing::compositeTextureV2 result(
            itkACVD, *_globals->getVolPkg(), uvMap, _radius, width, height,
            aFilterOption, aDirectionOption);

        // Setup rendering
        volcart::Rendering rendering;
        rendering.setTexture(result.texture());
        rendering.setMesh(itkACVD);

        _globals->setRendering(rendering);

    } catch (...) {
        if (cloudProblem) {
            _globals->setStatus(-1);

        } else {
            _globals->setStatus(-2);
        }
    };

    if (_globals->getStatus() == 0) {
        _globals->setStatus(1);
    }

    _globals->setProcessing(false);
}
