//----------------------------------------------------------------------------------------------------------------------------------------
// MyThread.cpp file for MyThread Class , (Implements QThread)
// Purpose: Run the Texture Functionality behind the scenes so that the GUI
// operates without interference
// Developer: Michael Royal - mgro224@g.uky.edu
// October 12, 2015 - Spring Semester 2016
// Last Updated 10/24/2016 by: Michael Royal

// Copyright 2015 (Brent Seales: Volume Cartography Research)
// University of Kentucky VisCenter
//----------------------------------------------------------------------------------------------------------------------------------------

#include "MyThread.h"
#include "core/io/OBJWriter.h"
#include "core/io/PLYReader.h"
#include "meshing/ITK2VTK.h"

namespace fs = boost::filesystem;

MyThread::MyThread(GlobalValues* globals)
{
    _globals = globals;
    _globals->setThreadStatus(ThreadStatus::Active);  // Status Running/Active
    this->start();
}

void MyThread::run()
{
    bool cloudProblem = false;

    try {
        double _radius = _globals->getRadius();

        fs::path meshName = _globals->getVolPkg()->getMeshPath();

        auto aFilterOption =
            static_cast<volcart::CompositeOption>(_globals->getTextureMethod());
        auto aDirectionOption = static_cast<volcart::DirectionOption>(
            _globals->getSampleDirection());

        // declare pointer to new Mesh object
        auto mesh = volcart::ITKMesh::New();

        // try to convert the ply to an ITK mesh
        volcart::io::PLYReader reader(meshName);
        try {
            reader.read();
            mesh = reader.getMesh();
        } catch (std::exception e) {
            cloudProblem = true;
            std::cerr << e.what() << std::endl;
            throw;
        }

        // Calculate sampling density
        double voxelsize = _globals->getVolPkg()->getVoxelSize();
        double sa = volcart::meshmath::SurfaceArea(mesh) *
                    (voxelsize * voxelsize) *
                    (0.001 * 0.001);  // convert vx^2 -> mm^2;
        double densityFactor = 50;
        auto numberOfVertices =
            static_cast<uint16_t>(std::round(densityFactor * sa));
        numberOfVertices = (numberOfVertices < CLEANER_MIN_REQ_POINTS)
                               ? CLEANER_MIN_REQ_POINTS
                               : numberOfVertices;

        // Convert to polydata
        auto vtkMesh = vtkSmartPointer<vtkPolyData>::New();
        volcart::meshing::ITK2VTK(mesh, vtkMesh);

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

        auto itkACVD = volcart::ITKMesh::New();
        volcart::meshing::VTK2ITK(Cleaner->GetOutput(), itkACVD);

        // ABF flattening
        std::cout << "Computing parameterization..." << std::endl;
        volcart::texturing::AngleBasedFlattening abf(itkACVD);
        // abf.setABFMaxIterations(5);
        abf.compute();

        // Get uv map
        volcart::UVMap uvMap = abf.getUVMap();
        auto width = static_cast<int>(std::ceil(uvMap.ratio().width));
        auto height = static_cast<int>(
            std::ceil(static_cast<double>(width) / uvMap.ratio().aspect));

        volcart::texturing::CompositeTextureV2 result(
            itkACVD, *_globals->getVolPkg(), uvMap, _radius, width, height,
            aFilterOption, aDirectionOption);

        // Setup rendering
        volcart::Rendering rendering;
        rendering.setTexture(result.texture());
        rendering.setMesh(itkACVD);

        _globals->setRendering(rendering);
        _globals->setThreadStatus(ThreadStatus::Successful);

    } catch (...) {
        if (cloudProblem) {
            _globals->setThreadStatus(ThreadStatus::CloudError);

        } else {
            _globals->setThreadStatus(ThreadStatus::Failed);
        }
    };
}
