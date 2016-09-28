// smoothNormals.cpp
// Abigail Coleman June 2015

#include "meshing/smoothNormals.h"

namespace volcart {
    namespace meshing {

        VC_MeshType::Pointer smoothNormals(VC_MeshType::Pointer input,
                                           double               radius) {

            std::cerr << "volcart::meshing::smoothNormals: radius " << radius << std::endl;
            // declare pointer to new Mesh object to be returned
            VC_MeshType::Pointer outputMesh = VC_MeshType::New();
            volcart::meshing::deepCopy(input, outputMesh);

            // Variables for normal smoothing
            cv::Vec3d neighborAvg;
            double neighborCount;

            // Use pointsLocator to find neighborhood within given radius
            typename VC_PointsLocatorType::Pointer pointsLocator = VC_PointsLocatorType::New();
            pointsLocator->SetPoints(input->GetPoints());
            pointsLocator->Initialize();
            typename VC_PointsLocatorType::NeighborsIdentifierType neighborhood;

            // Iterate over all of the cells to lay out the faces in the output texture
            for (VC_PointsInMeshIterator point = input->GetPoints()->Begin(); point != input->GetPoints()->End(); ++point) {
                std::cerr << "volcart::meshing::smoothNormals: " << point.Index() << "/" << input->GetNumberOfPoints() - 1 << "\r" << std::flush;

                // Empty our averaging variables
                if ( !neighborhood.empty() ) neighborhood.clear();
                neighborCount = 0;
                neighborAvg = cv::Vec3d(0,0,0);

                // Get the current normal and add it to the summed normal
                VC_PixelType currentNormal;
                input->GetPointData(point.Index(), &currentNormal);
                neighborAvg[0] += currentNormal[0];
                neighborAvg[1] += currentNormal[1];
                neighborAvg[2] += currentNormal[2];
                ++neighborCount;

                // find neighborhood for current point within radius
                pointsLocator->FindPointsWithinRadius(point->Value(), radius, neighborhood);

                // Sum the normals of the neighbors
                for (auto n_pt = neighborhood.begin(); n_pt != neighborhood.end(); ++n_pt) {
                    VC_PixelType neighborNormal;
                    input->GetPointData(*n_pt, &neighborNormal);

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
            std::cerr << std::endl;

            return outputMesh;
        }

    } // meshing
} // volcart
