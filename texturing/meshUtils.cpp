// meshUtils.h
// Abigail Coleman June 2015

#include "meshUtils.h"

VC_MeshType::Pointer smoothNormals ( VC_MeshType::Pointer  inputMesh,
                                     double                smoothingFactor ) {

    // declare pointer to new Mesh object to be returned
    VC_MeshType::Pointer  outputMesh = VC_MeshType::New();
    outputMesh = inputMesh; // copy faces, points, and old normals from input mesh

    // Define iterators
    VC_PointsInMeshIterator currentPoint, neighborPoint;
    currentPoint = inputMesh->GetPoints()->Begin();
    VC_PointsInMeshIterator pointsEnd = inputMesh->GetPoints()->End();
    
    // Variables for normal smoothing
    // cv::Vec3d neighborAvg;
    std::vector< long double > neighborAvg ( 3, 0 );
    double neighborCount, distance, pointID;

    // file to write old and new normals too
    std::ofstream myfile;
    myfile.open( "normals.txt" );

    // Use pointsLocator to find neighborhood within given radius
    typename VC_PointsLocatorType::Pointer pointsLocator = VC_PointsLocatorType::New();
    pointsLocator->SetPoints( inputMesh->GetPoints() );
    pointsLocator->Initialize();
    typename VC_PointsLocatorType::NeighborsIdentifierType neighborhood;

    // Iterate over all of the cells to lay out the faces in the output texture
    while ( currentPoint != pointsEnd )
    {
        std::cout << "Smoothing normals for point " << currentPoint.Index() << "/" << pointsEnd.Index() << "\r" << std::flush;

        VC_PointType p = currentPoint.Value();
        VC_PixelType currentNormal;
        inputMesh->GetPointData( currentPoint.Index(), &currentNormal );

        myfile << "Old: " << currentNormal[0] << ", " << currentNormal[1] << ", " << currentNormal[2] << "    ";

        neighborCount = 0;
        neighborAvg[0] = 0;
        neighborAvg[1] = 0;
        neighborAvg[2] = 0;
        neighborPoint = inputMesh->GetPoints()->Begin(); 
        
        // find neighborhood for current point (p) within radius
        pointsLocator->FindPointsWithinRadius( p, smoothingFactor, neighborhood );
        neighborCount = neighborhood.size();

        for ( int i = 0; i < neighborCount; ++i ) {
            pointID = neighborhood[i];
            VC_PointType p2 = inputMesh->GetPoint( pointID );
            VC_PixelType neighborNormal;
            inputMesh->GetPointData( neighborPoint.Index(), &neighborNormal );

            neighborAvg[0] += neighborNormal[0];
            neighborAvg[1] += neighborNormal[1];
            neighborAvg[2] += neighborNormal[2];
        }
        if( neighborCount > 0) {
            // Calculate neighborhood's normal average and smooth
            currentNormal[0] = neighborAvg[0] / neighborCount;
            currentNormal[1] = neighborAvg[1] / neighborCount;
            currentNormal[2] = neighborAvg[2] / neighborCount;
            outputMesh->SetPointData( currentPoint.Index(), currentNormal );
            myfile << "New: " << currentNormal[0] << ", " << currentNormal[1] << ", " << currentNormal[2] << "    ";
            myfile << "Neighbor Count: " << neighborCount << "\n";
        }

        ++currentPoint;
    }
    std::cout << std::endl;
    myfile.close();
        
    return outputMesh;
}
