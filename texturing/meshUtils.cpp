// meshUtils.h
// Abigail Coleman June 2015

#include "meshUtils.h"
#include <opencv2/opencv.hpp>

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
    PointsIterator pointsIterator, pointsIterator2;
    pointsIterator = inputMesh->GetPoints()->Begin();
    PointsIterator end = inputMesh->GetPoints()->End();
    
    // Variables for normal smoothing
    cv::Vec3d neighborAvg;
    double neighborCount, distance;

    // Iterate over all of the cells to lay out the faces in the output texture
    while ( pointsIterator != end )
    {
        std::cout << "Smoothing normals for point " << pointsIterator.Index() << "/" << end.Index() << "\r" << std::flush;

        MeshType::PointType p = pointsIterator.Value();
        MeshType::PixelType normal;
	inputMesh->GetPointData( pointsIterator.Index(), &normal );

        neighborCount = 0;
        neighborAvg[0] = 0;
        neighborAvg[1] = 0;
        neighborAvg[2] = 0;
        pointsIterator2 = inputMesh->GetPoints()->Begin();
        
	// Generate neighborhood for current point (p)
	while ( pointsIterator2 != end )
        {
            MeshType::PointType p2 = pointsIterator2.Value();
	    MeshType::PixelType normal2;
            inputMesh->GetPointData( pointsIterator.Index(), &normal2 );

	    // Calculate distance of neighbor candidate to initial point (p)
            distance = pow((p2[0]-p[0]),2) + pow((p2[1]-p[1]),2) +  pow((p2[2]-p[2]),2);
            if ( distance < pow(smoothingFactor,2) ) {
            	neighborAvg[0] = neighborAvg[0] + normal2[0];
               	neighborAvg[1] = neighborAvg[1] + normal2[1];
		neighborAvg[2] = neighborAvg[2] + normal2[2];
		++neighborCount;
            }

            ++pointsIterator2;
        }
	if( neighborCount > 0) {
            // Calculate neighborhood's normal average and smooth
            neighborAvg[0] = neighborAvg[0] / neighborCount;
            neighborAvg[1] = neighborAvg[1] / neighborCount;
            neighborAvg[2] = neighborAvg[2] / neighborCount;
	    // Update current points normal to smoothed normal
            normal[0] = neighborAvg[0];
            normal[1] = neighborAvg[1];
            normal[2] = neighborAvg[2];
            outputMesh->SetPointData( pointsIterator.Index(), normal );
        }

	++pointsIterator;
    }
    std::cout << std::endl;
	
    return outputMesh;
}
