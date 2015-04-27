// main.cpp
// Abigail Coleman Feb. 2015

#include <iostream>
#include <fstream>
#include <string>

#include <opencv2/opencv.hpp>

#include "texturingUtils.h"
#include "volumepkg.h"
#include "checkPtInTriangleUtil.h"

#include <itkMesh.h>
#include <itkRGBPixel.h>
#include <itkTriangleCell.h>

#include "UPointMapping.h"

int main(int argc, char* argv[])
{
	typedef itk::Vector< double, 3 >    PixelType;	// A vector to hold the normals along with the points of each vertice in the mesh
	const unsigned int Dimension = 3;	// Need a 3 Dimensional Mesh

	// declare Mesh object using template parameters 
	typedef itk::Mesh< PixelType, Dimension >   MeshType;
	
	// declare pointer to new Mesh object
	MeshType::Pointer  mesh = MeshType::New();

	if( argc < 3 ) {
		std::cout << "Usage: GenMesh mesh.ply volpkg" << std::endl;
        	exit( -1 );
    	}

	// open ply file
	std::string fileName( argv[ 1 ] );
	std::ifstream plyFile( fileName.c_str() );
	if ( !plyFile.is_open() ) {
		std::cerr << "Open file " << fileName << " failed." << std::endl;
		return false;
	}

	// parse ply file
	// REVISIT - quick and dirty read, specific for the file written by write_mesh() in simulation/simulation.cpp
        std::string line;

        // read header
        int numVertices, numFaces;
        const int NUM_LINES_IN_HEADER = 21;
        const int NUM_VERTEX_POS = 15;
        const int NUM_FACE_POS   = 13;
        for ( int i = 0; i < NUM_LINES_IN_HEADER; ++i ) 
	{
                getline( plyFile, line );
                if ( line.find( "element vertex " ) != std::string::npos ) {
                        numVertices = std::atoi( line.substr( NUM_VERTEX_POS ).c_str() );
                }
                if ( line.find( "element face " ) != std::string::npos ) {
                        numFaces = std::atoi( line.substr( NUM_FACE_POS ).c_str() );
                }
        }
	// REVISIT - for debug
        std::cout << "# of vertices to be read: " << numVertices << std::endl;
        std::cout << "# of faces to be read: " << numFaces << std::endl;	


	// get the dimensions of the mesh from the header
        int width, height;
        plyFile >> width >> height;
        std::cout << "width: " << width << std::endl;
        std::cout << "height: " << height << std::endl;

	// read vertices
	double x, y, z, nx, ny, nz, s, t;
	int red, green, blue;
        for ( int i = 0; i < numVertices; ++i ) 
	{
		MeshType::PointType p;
        	MeshType::PixelType n;
                plyFile >> x >> y >> z >> nx >> ny >> nz >> s >> t >> red >> green >> blue;
		p[0] = x;
		p[1] = y;
		p[2] = z;
		n[0] = nx;
		n[1] = ny;
		n[2] = nz;
		mesh->SetPoint( i, p );
		mesh->SetPointData( i, n );
        }	
	
	
	// read faces
        int temp, p1, p2, p3;
	typedef MeshType::CellType		  CellType;
	typedef itk::TriangleCell< CellType >     TriangleType;
	CellType::CellAutoPointer cellpointer;
	for ( int i = 0; i < numFaces; ++i ) 
	{
                plyFile >> temp >> p1 >> p2 >> p3;

		cellpointer.TakeOwnership( new TriangleType );
  		cellpointer->SetPointId( 0, p1 );
  		cellpointer->SetPointId( 1, p2 );
  		cellpointer->SetPointId( 2, p3 );
  		mesh->SetCell( i, cellpointer );
	}

	// generate 2D coordinates
	typedef CellType::PointIdIterator     PointsIterator2;
        typedef MeshType::CellsContainer::Iterator  CellIterator;
        CellIterator  cellIterator = mesh->GetCells()->Begin();
        CellIterator  cellEnd          = mesh->GetCells()->End();
	std::vector< cv::Vec3d > my3DPoints;    // 3D vector to hold 3D points
        std::vector< cv::Vec3d > my2DPoints;    // 3D vector to hold 2D points along with a 1 in the z coordinate
        cv::Vec3d my3DPoint;
        cv::Vec3d my2DPoint;
        cv::Mat myH( 3, 3, CV_64F );            // homography matrix
	cv::Mat M( 1024, 1024, CV_16U );
	unsigned long id;
	double u, v, Max = 0, Min = 8000;
	int meshx, meshy;
        
	//store volume in matrix for interpolation
	VolumePkg vpkg = VolumePkg( argv[ 2 ] );
	std::vector< cv::Mat > aImgVol;
	int aNumSlices = vpkg.getNumberOfSlices();
 	for ( int i = 0; i < aNumSlices; ++i ) 
	{
 		aImgVol.push_back( vpkg.getSliceAtIndex( i ).clone() );
 	}

        while( cellIterator != cellEnd )
        {
                CellType * cell = cellIterator.Value();
                PointsIterator2  pointsIterator = cell->PointIdsBegin();
                PointsIterator2 pointEnd = cell->PointIdsEnd();
		cellpointer.TakeOwnership( new TriangleType );
                while( pointsIterator != pointEnd)
                {
                 	id = *pointsIterator;
                        MeshType::PointType p = mesh->GetPoint(id);
			MeshType::PixelType normal;
                        mesh->GetPointData( id, &normal );
			meshx = id % width;
			meshy = (id - meshx) / width;
			u =  1024 * meshx / width;		// u
                        v =  1024 * meshy / height;		// v
			//std::cout << "U: " << u << " V: " << v <<  " normal x: " << normal[1] << std::endl;
                        M.at< double >( u, v ) = FilterIntersection( cv::Vec3f( p[0], p[1], p[2] ), aImgVol );
			//std::cout << "id: " << id << " | " << p[0] << " | " << p[1] << " | " << p[2] << std::endl;
			++pointsIterator;

			if(M.at< double >( u, v ) > Max)
				Max = M.at< double >( u, v );
			else if(M.at< double >( u, v ) < Min)
				Min = M.at< double >( u, v );

			/* Store points to be used in calculating Homography matrix
			my3DPoint = cv::Vec3d( p[0], p[1], p[2] );
			my2DPoint = cv::Vec3d( u, v, 1.0 );
			my3DPoints.push_back( my3DPoint );
			my2DPoints.push_back( my2DPoint );*/
                }
		/* Generate Homography matrix for current cell/face
		CalcHomographyFromPoints( my3DPoints, my2DPoints, myH );
		my3DPoints.clear();
		my2DPoints.clear();*/

                ++cellIterator;
        }

	// interpolate intensity values for the rest of the matrix M
	cellIterator = mesh->GetCells()->Begin();
	int k = 0;
	for(int i = 0; i < 1024; ++i)
	{
		for(int j = 0; j < 1024; ++j)
		{
			while( cellIterator != cellEnd )
       	 		{
                		CellType * cell = cellIterator.Value();
                		PointsIterator2 pointsIterator = cell->PointIdsBegin();
                		PointsIterator2 pointEnd = cell->PointIdsEnd();
                		cellpointer.TakeOwnership( new TriangleType );
 				checkPtInTriangleUtil::Point A, B, C, P;
				P.data[0] = i; P.data[1] = j; P.data[2] = 0.0;
	               		while( pointsIterator != pointEnd)
            			{
                        		id = *pointsIterator;
                        		MeshType::PointType p = mesh->GetPoint(id);
					u =  1024 * (id % width) / width;       	// u
                        		v =  1024 * ((id - u) / width) / height;        // v
					A.data[0] = u; A.data[1] = v; A.data[2] = 0.0;
					++id;
					u =  1024 * (id % width) / width;               // u
                                        v =  1024 * ((id - u) / width) / height;        // v
					B.data[0] = u; B.data[1] = v; B.data[2] = 0.0;
					++id;
                                        u =  1024 * (id % width) / width;               // u
                                        v =  1024 * ((id - u) / width) / height;        // v
					C.data[0] = u; C.data[1] = v; C.data[2] = 0.0;
					pointsIterator = pointsIterator + 3;	
				}
				//printf( "P is %s of the triangle ABC.\n", IsPtInTriangle( P, A, B, C ) ? "inside" : "outside" );
				bool inTriangle = IsPtInTriangle( P, A, B, C );
				if( inTriangle == true){
					std::cout << "Point is inside triangle" << std::endl;
				}
				++cellIterator;
			}
		}
	}
	

	//cv::Mat B;
	//std::cout << "Max: " << Max << " Min: " << Min << std::endl;
	//M.convertTo(B, CV_16U);
	//cvtColor(M, M, CV_BGR2Luv);
	cv::imwrite("filterIntersection.tiff", M);

	plyFile.close();

	return 0;
} // end main

