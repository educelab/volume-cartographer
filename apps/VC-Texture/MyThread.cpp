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

#include <cmath>

#include "MyThread.hpp"

#include "vc/core/io/OBJWriter.hpp"
#include "vc/core/io/PLYReader.hpp"
#include "vc/core/util/MeshMath.hpp"
#include "vc/meshing/ACVD.hpp"
#include "vc/meshing/ITK2VTK.hpp"
#include "vc/meshing/OrderedPointSetMesher.hpp"
#include "vc/texturing/AngleBasedFlattening.hpp"
#include "vc/texturing/CompositeTexture.hpp"
#include "vc/texturing/PPMGenerator.hpp"

namespace vc = volcart;

MyThread::MyThread(GlobalValues* globals)
{
    _globals = globals;
    _globals->setThreadStatus(ThreadStatus::Active);  // Status Running/Active
    this->start();
}

void MyThread::run()
{
    try {
        double _radius = _globals->getRadius();

        auto aFilterOption =
            static_cast<volcart::texturing::CompositeTexture::Filter>(
                _globals->getTextureMethod());
        auto aDirectionOption =
            static_cast<volcart::Direction>(_globals->getSampleDirection());

        ///// Load and resample the segmentation /////
        if (!_globals->getActiveSegmentation()->hasPointSet()) {
            std::cerr << "VC::message: Empty pointset" << std::endl;
            _globals->setThreadStatus(ThreadStatus::CloudError);
            return;
        }

        // Load the cloud
        auto cloud = _globals->getActiveSegmentation()->getPointSet();

        // Only do it if we have more than one iteration of segmentation
        if (cloud.height() <= 1) {
            std::cerr << "VC::message: Cloud height <= 1. Nothing to mesh."
                      << std::endl;
            return;
        }

        // Mesh the point cloud
        volcart::meshing::OrderedPointSetMesher mesher;
        mesher.setPointSet(cloud);

        // declare pointer to new Mesh object
        auto mesh = mesher.compute();

        // Calculate sampling density
        auto voxelsize = _globals->getVolPkg()->volume()->voxelSize();
        auto sa = vc::meshmath::SurfaceArea(mesh) * std::pow(voxelsize, 2) *
                  (0.001 * 0.001);
        double densityFactor = 50;
        auto numVerts = static_cast<uint16_t>(std::round(densityFactor * sa));
        numVerts = (numVerts < CLEANER_MIN_REQ_POINTS) ? CLEANER_MIN_REQ_POINTS
                                                       : numVerts;

        // Convert to polydata
        auto vtkMesh = vtkSmartPointer<vtkPolyData>::New();
        volcart::meshing::ITK2VTK(mesh, vtkMesh);

        // Decimate using ACVD
        std::cout << "Resampling mesh..." << std::endl;
        auto acvdMesh = vtkSmartPointer<vtkPolyData>::New();
        volcart::meshing::ACVD(vtkMesh, acvdMesh, numVerts);

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
        auto width = static_cast<size_t>(std::ceil(uvMap.ratio().width));
        auto height = static_cast<size_t>(
            std::ceil(static_cast<double>(width) / uvMap.ratio().aspect));

        volcart::texturing::PPMGenerator ppmGen(height, width);
        ppmGen.setMesh(itkACVD);
        ppmGen.setUVMap(uvMap);
        ppmGen.compute();

        volcart::texturing::CompositeTexture result;
        result.setPerPixelMap(ppmGen.getPPM());
        result.setVolume(_globals->getVolPkg()->volume());
        result.setFilter(aFilterOption);
        result.setSamplingRadius(_radius);
        result.setSamplingDirection(aDirectionOption);
        result.compute();

        // Setup rendering
        volcart::Rendering rendering;
        rendering.setTexture(result.getTexture());
        rendering.setMesh(itkACVD);

        _globals->setRendering(rendering);
        _globals->setThreadStatus(ThreadStatus::Successful);

    } catch (...) {
        _globals->setThreadStatus(ThreadStatus::Failed);
    };
}
