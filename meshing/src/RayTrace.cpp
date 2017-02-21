#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wconversion"
#pragma clang diagnostic ignored "-Wold-style-cast"

//
// Created by Media Team on 8/12/15.
//
/**@file RayTrace.cpp */
#include <cstdio>

#include <vtkCellData.h>
#include <vtkOBBTree.h>
#include <vtkPolyDataNormals.h>
#include <vtkSmartPointer.h>

#include "vc/meshing/ITK2VTK.hpp"
#include "vc/meshing/RayTrace.hpp"

// Parameters used to create cylindrical ray tracing
#define PI_X2 (2 * 3.1415926)
#define OUT_X 2000
#define D_THETA (PI_X2 / OUT_X)

namespace volcart
{
namespace meshing
{

// returns a vector of vectors that holds the points of intersections and the
// corresponding normals
std::vector<cv::Vec6f> RayTrace(
    const ITKMesh::Pointer& itkMesh,
    int aTraceDir,
    int /*width*/,
    int /*height*/,
    std::map<int, cv::Vec2d>& uvMap)
{

    // Essential data structure to return points and normals
    std::vector<cv::Vec6f> intersections;

    // Convert the itk mesh to a vtk mesh
    auto vtkMesh = vtkPolyData::New();
    volcart::meshing::ITK2VTK(itkMesh, vtkMesh);

    // Get the bounds of the mesh
    std::array<double, 6> bounds{};
    vtkMesh->GetBounds(bounds.data());

    // Generate normals for the cells of the mesh
    auto calcNormals = vtkSmartPointer<vtkPolyDataNormals>::New();
    calcNormals->SetInputData(vtkMesh);
    calcNormals->ComputeCellNormalsOn();
    calcNormals->Update();
    vtkMesh = calcNormals->GetOutput();

    // Create vtk OBBTree
    auto obbTree = vtkSmartPointer<vtkOBBTree>::New();
    obbTree->SetDataSet(vtkMesh);

    // Increases BuildLocator time, but its worth it for large meshes
    obbTree->SetNumberOfCellsPerNode(2);
    obbTree->SetMaxLevel(48);
    std::cout << "volcart::RayTrace :: Building locator..." << std::endl;
    obbTree->BuildLocator();

    auto intersectPoints = vtkSmartPointer<vtkPoints>::New();
    auto intersectCells = vtkSmartPointer<vtkIdList>::New();

    // Calculate the origin by averaging the bounds of each coordinate
    cv::Vec3f origin;
    origin(0) = (bounds[0] + bounds[1]) / 2;
    origin(1) = (bounds[2] + bounds[3]) / 2;

    // Point ID counter for intersecting points
    int pointID = 0;

    // For each slice/row generate rays and interpolate new points
    for (int z = static_cast<int>(bounds[4]); z < static_cast<int>(bounds[5]);
         ++z) {
        std::cout << "\rvolcart::RayTrace :: Ray tracing for Z index: " << z
                  << "/" << static_cast<int>(bounds[5] - 1) << std::flush;

        origin(2) = z;  // update the z-component of the origin

        // Cylindrical ray generation
        int ycount = 0;     // counter for the number of rays per slice
        double radian = 0;  // the angle for the ray
        double r = 0;
        while (r < PI_X2) {
            // Calculate the ray according to ray tracing direction
            if (aTraceDir == 1) {
                radian -= D_THETA;  // counterclockwise
            } else {
                radian += D_THETA;  // clockwise (default)
            }

            // Calculate direction of ray according to current degree of
            // rotation along the cylinder
            cv::Vec3f direction(cos(radian), sin(radian), 0);
            cv::normalize(direction);

            // Create a second point along the ray using the origin and
            // direction
            cv::Vec3f endPoint = origin + 400 * direction;

            std::array<double, 3> start = {origin[0], origin[1], origin[2]};
            std::array<double, 3> end = {endPoint[0], endPoint[1], endPoint[2]};

            obbTree->IntersectWithLine(
                start.data(), end.data(), intersectPoints, intersectCells);

            if (intersectPoints->GetNumberOfPoints() > 0) {
                cv::Vec6f point;
                auto intersectPoint = intersectPoints->GetPoint(0);
                point[0] = intersectPoint[0];
                point[1] = intersectPoint[1];
                point[2] = intersectPoint[2];

                auto intersectCell = intersectCells->GetId(0);
                auto normals = vtkMesh->GetCellData()->GetNormals();
                point[3] = normals->GetTuple(intersectCell)[0];
                point[4] = normals->GetTuple(intersectCell)[1];
                point[5] = normals->GetTuple(intersectCell)[2];
                intersections.push_back(point);

                // Add the uv coordinates into our map at the point index
                // specified
                cv::Vec2d uv(ycount, z - bounds[4]);
                uvMap.insert({pointID, uv});

                ++pointID;  // Increment our point id counter
            }               // if intersection

            r += D_THETA;
            ycount++;
        }  // for each ray

    }  // for each z
    std::cout << std::endl;
    return intersections;
}
}
}

#pragma clang diagnostic pop
