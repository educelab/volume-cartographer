// meshUtils.h
// Abigail Coleman June 2015

#include "meshUtils.h"

itk::Mesh<::itk::Vector<double, 3>, 3>::Pointer smoothNormals ( itk::Mesh<::itk::Vector<double, 3>, 3>::Pointer inputMesh, 
                                                                double                                          smoothingFactor ) {
    typedef itk::Vector< double, 3 >  PixelType;  // A vector to hold the normals along with the points of each vertice in the mesh
    const unsigned int Dimension = 3;   // Need a 3 Dimensional Mesh

    // declare Mesh object using template parameters 
    typedef itk::Mesh< PixelType, Dimension >   MeshType;

    // declare pointer to new Mesh object to be returned
    MeshType::Pointer  outputMesh = MeshType::New();
    outputMesh = inputMesh; // copy faces, points, and old normals from input mesh

    // Define iterators
    typedef MeshType::PointsContainer::Iterator     PointsIterator;
    PointsIterator currentPoint, neighborPoint;
    currentPoint = inputMesh->GetPoints()->Begin();
    PointsIterator pointsEnd = inputMesh->GetPoints()->End();
    
    // Variables for normal smoothing
    // cv::Vec3d neighborAvg;
    std::vector< long double > neighborAvg ( 3, 0 );
    double neighborCount, distance, pointID;

    // file to write old and new normals too
    std::ofstream myfile;
    myfile.open( "normals.txt" );

    // Use pointsLocator to find neighborhood within given radius
    typedef MeshType::PointsContainer PointsContainerType;
    typedef itk::PointsLocator<PointsContainerType> PointsLocatorType;
    typename PointsLocatorType::Pointer pointsLocator = PointsLocatorType::New();
    pointsLocator->SetPoints( inputMesh->GetPoints() );
    pointsLocator->Initialize();
    typename PointsLocatorType::NeighborsIdentifierType neighborhood;

    // Iterate over all of the cells to lay out the faces in the output texture
    while ( currentPoint != pointsEnd )
    {
        std::cout << "Smoothing normals for point " << currentPoint.Index() << "/" << pointsEnd.Index() << "\r" << std::flush;

        MeshType::PointType p = currentPoint.Value();
        MeshType::PixelType currentNormal;
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
            MeshType::PointType p2 = inputMesh->GetPoint( pointID );
            MeshType::PixelType neighborNormal;
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
