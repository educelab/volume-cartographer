//
// Created by Media Team on 8/12/15.
//
/**@file rayTrace.cpp */
#include "meshing/rayTrace.h"
#include <cstdio>
#include <vtkOBBTree.h>
#include <vtkPolyDataNormals.h>
#include <vtkSmartPointer.h>
#include "meshing/itk2vtk.h"

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
std::vector<cv::Vec6f> rayTrace(
    ITKMesh::Pointer itkMesh,
    int aTraceDir,
    int width,
    int height,
    std::map<int, cv::Vec2d> uvMap)
{

    // Essential data structure to return points and normals
    std::vector<cv::Vec6f> intersections;

    // Convert the itk mesh to a vtk mesh
    vtkPolyData* vtkMesh = vtkPolyData::New();
    volcart::meshing::itk2vtk(itkMesh, vtkMesh);

    // Get the bounds of the mesh
    double bounds[6];
    vtkMesh->GetBounds(bounds);

    // Set ray width and height, used for texturing
    height = (int)(bounds[5] - bounds[4]);
    width = OUT_X;

    // Generate normals for the cells of the mesh
    vtkSmartPointer<vtkPolyDataNormals> calcNormals =
        vtkSmartPointer<vtkPolyDataNormals>::New();
    calcNormals->SetInputData(vtkMesh);
    calcNormals->ComputeCellNormalsOn();
    calcNormals->Update();
    vtkMesh = calcNormals->GetOutput();

    // Create vtk OBBTree
    vtkSmartPointer<vtkOBBTree> obbTree = vtkSmartPointer<vtkOBBTree>::New();
    obbTree->SetDataSet(vtkMesh);
    obbTree->SetNumberOfCellsPerNode(
        2);  // Increases BuildLocator time, but its worth it for large meshes
    obbTree->SetMaxLevel(48);
    std::cout << "volcart::rayTrace :: Building locator..." << std::endl;
    obbTree->BuildLocator();

    vtkSmartPointer<vtkPoints> intersectPoints =
        vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkIdList> intersectCells =
        vtkSmartPointer<vtkIdList>::New();

    // Calculate the origin by averaging the bounds of each coordinate
    cv::Vec3f origin;
    origin(0) = (bounds[0] + bounds[1]) / 2;
    origin(1) = (bounds[2] + bounds[3]) / 2;

    // Point ID counter for intersecting points
    int pointID = 0;

    // For each slice/row generate rays and interpolate new points
    for (int z = (int)bounds[4]; z < (int)bounds[5]; ++z) {
        std::cout << "\rvolcart::rayTrace :: Ray tracing for Z index: " << z
                  << "/" << (int)bounds[5] - 1 << std::flush;

        origin(2) = z;  // update the z-component of the origin

        // Cylindrical ray generation
        int ycount = 0;     // counter for the number of rays per slice
        double radian = 0;  // the angle for the ray
        for (double r = 0; r < PI_X2; r += D_THETA, ycount++) {
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
            cv::Vec3f end_point = origin + 400 * direction;

            double start[3] = {origin[0], origin[1], origin[2]};
            double end[3] = {end_point[0], end_point[1], end_point[2]};

            obbTree->IntersectWithLine(
                start, end, intersectPoints, intersectCells);

            if (intersectPoints->GetNumberOfPoints() > 0) {
                cv::Vec6f point;
                point[0] = intersectPoints->GetPoint(0)[0];
                point[1] = intersectPoints->GetPoint(0)[1];
                point[2] = intersectPoints->GetPoint(0)[2];
                point[3] = vtkMesh->GetCellData()->GetNormals()->GetTuple(
                    intersectCells->GetId(0))[0];
                point[4] = vtkMesh->GetCellData()->GetNormals()->GetTuple(
                    intersectCells->GetId(0))[1];
                point[5] = vtkMesh->GetCellData()->GetNormals()->GetTuple(
                    intersectCells->GetId(0))[2];
                intersections.push_back(point);

                // Add the uv coordinates into our map at the point index
                // specified
                cv::Vec2d uv(ycount, z - bounds[4]);
                uvMap.insert({pointID, uv});

                ++pointID;  // Increment our point id counter
            }               // if intersection

        }  // for each ray

    }  // for each z
    std::cout << std::endl;
    return intersections;
}  // rayTrace
}  // namespace meshing
}  // namespace volcart
