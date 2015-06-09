// meshUtils.h
// Abigail Coleman June 2015

#include "meshUtils.h"

itk::Mesh<::itk::Vector<double, 3>, 3>::Pointer smoothNormals ( itk::Mesh<::itk::Vector<double, 3>, 3>::Pointer inputMesh, 
								double						smoothingFactor ) {
    typedef itk::Vector< double, 3 >  PixelType;  // A vector to hold the normals along with the points of each vertice in the mesh
    const unsigned int Dimension = 3;   // Need a 3 Dimensional Mesh

    // declare Mesh object using template parameters 
    typedef itk::Mesh< PixelType, Dimension >   MeshType;

    // declare pointer to new Mesh object to be returned
    MeshType::Pointer  outputMesh = MeshType::New();

    // Define iterators
    typedef MeshType::PointsContainer::Iterator     PointsIterator;
    PointsIterator currentPoint, neighborPoint;
    currentPoint = inputMesh->GetPoints()->Begin();
    PointsIterator pointsEnd = inputMesh->GetPoints()->End();
    
    // Variables for normal smoothing
    cv::Vec3d neighborAvg;
    double neighborCount, distance;

    // file to write old and new normals too
    std::ofstream myfile;
    myfile.open( "normals.txt" );

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
        
	// Generate neighborhood for current point (p)
	while ( neighborPoint != pointsEnd )
        {
            MeshType::PointType p2 = neighborPoint.Value();
	    MeshType::PixelType neighborNormal;
            inputMesh->GetPointData( neighborPoint.Index(), &neighborNormal );

	    // Calculate distance of neighbor candidate to initial point (p)
            distance = pow((p2[0]-p[0]),2) + pow((p2[1]-p[1]),2) +  pow((p2[2]-p[2]),2);
            if ( distance < pow(smoothingFactor,2) ) {
            	neighborAvg[0] += neighborNormal[0];
               	neighborAvg[1] += neighborNormal[1];
		neighborAvg[2] += neighborNormal[2];
		++neighborCount;
            }

            ++neighborPoint;
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
