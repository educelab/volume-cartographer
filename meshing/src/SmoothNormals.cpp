// SmoothNormals.cpp
// Abigail Coleman June 2015

/** @file SmoothNormals.cpp*/
#include <fstream>
#include <iostream>
#include <vector>

#include <opencv2/core.hpp>

#include "vc/meshing/SmoothNormals.hpp"

namespace volcart::meshing
{

auto SmoothNormals(const ITKMesh::Pointer& input, double radius)
    -> ITKMesh::Pointer
{
    // declare pointer to new Mesh object to be returned
    auto outputMesh = ITKMesh::New();
    DeepCopy(input, outputMesh);

    // Variables for normal smoothing
    cv::Vec3d neighborAvg;
    double neighborCount;

    // Use pointsLocator to find neighborhood within given radius
    auto pointsLocator = ITKPointsLocator::New();
    pointsLocator->SetPoints(input->GetPoints());
    pointsLocator->Initialize();
    typename ITKPointsLocator::NeighborsIdentifierType neighborhood;

    // Iterate over all of the cells to lay out the faces in the output texture
    for (auto point = input->GetPoints()->Begin();
         point != input->GetPoints()->End(); ++point) {

        // Empty our averaging variables
        if (!neighborhood.empty()) {
            neighborhood.clear();
        }
        neighborCount = 0;
        neighborAvg = cv::Vec3d(0, 0, 0);

        // Get the current normal and add it to the summed normal
        ITKPixel currentNormal;
        input->GetPointData(point.Index(), &currentNormal);
        neighborAvg[0] += currentNormal[0];
        neighborAvg[1] += currentNormal[1];
        neighborAvg[2] += currentNormal[2];
        ++neighborCount;

        // find neighborhood for current point within radius
        pointsLocator->FindPointsWithinRadius(
            point->Value(), radius, neighborhood);

        // Sum the normals of the neighbors
        for (auto nb : neighborhood) {
            ITKPixel neighborNormal;
            input->GetPointData(nb, &neighborNormal);

            neighborAvg[0] += neighborNormal[0];
            neighborAvg[1] += neighborNormal[1];
            neighborAvg[2] += neighborNormal[2];
            ++neighborCount;
        }

        // Average the sum normal
        currentNormal[0] = neighborAvg[0] / neighborCount;
        currentNormal[1] = neighborAvg[1] / neighborCount;
        currentNormal[2] = neighborAvg[2] / neighborCount;
        outputMesh->SetPointData(point.Index(), currentNormal);
    }

    return outputMesh;
}
}  // namespace volcart::meshing
