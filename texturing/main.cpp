// main.cpp
// Abigail Coleman Feb. 2015

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>

#include "texturingUtils.h"
#include "volumepkg.h"
#include "checkPtInTriangleUtil.h"
#include "plyHelper.h"

#include <itkMesh.h>
#include <itkRGBPixel.h>
#include <itkTriangleCell.h>

#include "UPointMapping.h"

int main(int argc, char* argv[])
{
    typedef itk::Vector< double, 3 >    PixelType;  // A vector to hold the normals along with the points of each vertice in the mesh
    const unsigned int Dimension = 3;   // Need a 3 Dimensional Mesh

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

    int meshWidth = -1;
    int meshHeight = -1;

    // try to convert the ply to an ITK mesh
    int *mWpoint, *mHpoint;
    mWpoint = &meshWidth;
    mHpoint = &meshHeight;
    if (!ply2itkmesh(fileName, mesh, mWpoint, mHpoint)){
        exit( -1 );
    };

    // Define iterators
    typedef CellType::PointIdIterator     PointsIterator2;
    typedef MeshType::CellsContainer::Iterator  CellIterator;

    // Misc. vectors
    std::vector< cv::Vec3d > my3DPoints;    // 3D vector to hold 3D points
    std::vector< cv::Vec3d > my2DPoints;    // 3D vector to hold 2D points along with a 1 in the z coordinate
    cv::Vec3d my3DPoint;
    cv::Vec3d my2DPoint;

    // Homography matrix
    cv::Mat myH( 3, 3, CV_64F );

    // Matrix to store the output texture
    int textureW = meshWidth;
    int textureH = meshHeight;
    cv::Mat outputTexture( textureH, textureW, CV_16UC1 );

    // pointID == point's position in 1D list of points
    // [meshX, meshY] == point's position if list was a 2D matrix
    // [u, v] == point's position in the output matrix
    unsigned long pointID, meshX, meshY;
    double u, v;

    double Max = 0, Min = 8000;
        
    // Load the slices from the volumepkg
    VolumePkg vpkg = VolumePkg( argv[ 2 ] );
    std::vector< cv::Mat > aImgVol;
    int aNumSlices = vpkg.getNumberOfSlices();
    for ( int i = 0; i < aNumSlices; ++i ) 
    {
        std::cout << "\rLoading slice: " << i << std::flush;
        aImgVol.push_back( vpkg.getSliceAtIndex( i ).clone() );
    }
    std::cout << std::endl;

    // Initialize iterators
    CellIterator  cellIterator = mesh->GetCells()->Begin();
    CellIterator  cellEnd      = mesh->GetCells()->End();

    // Iterate over all of the cells
    CellType * cell;
    PointsIterator2 pointsIterator;

    while( cellIterator != cellEnd )
    {
        // Link the pointer to our current cell
        cell = cellIterator.Value();
        //cell.TakeOwnership( new TriangleType );

        // Iterate over the vertices of the current cell
        pointsIterator = cell->PointIdsBegin();
        while( pointsIterator != cell->PointIdsEnd())
        {
            pointID = *pointsIterator;

            MeshType::PointType p = mesh->GetPoint(pointID);
            MeshType::PixelType normal;
            mesh->GetPointData(pointID, &normal );

            // Calculate the point's [meshX, meshY] position based on its pointID
            meshX = pointID % meshWidth;
            meshY = (pointID - meshX) / meshWidth;

            // Calculate the point's pixel position in the output texture
            u =  (double) textureW * (double) meshX / (double) meshWidth;
            v =  (double) textureH * (double) meshY / (double) meshHeight;

            // Fill in the output pixel with a value
			// cv::Mat.at uses (row, column)
            double value = FilterIntersection( cv::Vec3f( p[0], p[1], p[2] ), aImgVol );
            outputTexture.at< unsigned short >( v, u ) = (unsigned short) value;

    //      if(M.at< double >( u, v ) > Max)
    //          Max = M.at< double >( u, v );
    //      else if(M.at< double >( u, v ) < Min)
    //          Min = M.at< double >( u, v );

            /* Store points to be used in calculating Homography matrix
            my3DPoint = cv::Vec3d( p[0], p[1], p[2] );
            my2DPoint = cv::Vec3d( u, v, 1.0 );
            my3DPoints.push_back( my3DPoint );
            my2DPoints.push_back( my2DPoint );*/

            ++pointsIterator;
        }

        /* Generate Homography matrix for current cell/face
        CalcHomographyFromPoints( my3DPoints, my2DPoints, myH );
        my3DPoints.clear();
        my2DPoints.clear();*/

        ++cellIterator;
    }

    // interpolate intensity values for the rest of the matrix M
//  cellIterator = mesh->GetCells()->Begin();
//  int k = 0;
//  for(int i = 0; i < 1024; ++i)
//  {
//      for(int j = 0; j < 1024; ++j)
//      {
//          while( cellIterator != cellEnd )
//                  {
//                      CellType * cell = cellIterator.Value();
//                      PointsIterator2 pointsIterator = cell->PointIdsBegin();
//                      PointsIterator2 pointEnd = cell->PointIdsEnd();
//                      cellpointer.TakeOwnership( new TriangleType );
//              checkPtInTriangleUtil::Point A, B, C, P;
//              P.data[0] = i; P.data[1] = j; P.data[2] = 0.0;
//                      while( pointsIterator != pointEnd)
//                      {
//                              id = *pointsIterator;
//                              MeshType::PointType p = mesh->GetPoint(id);
//                  u =  1024 * (id % meshWidth) / meshWidth;           // u
//                              v =  1024 * ((id - u) / width) / height;        // v
//                  A.data[0] = u; A.data[1] = v; A.data[2] = 0.0;
//                  ++id;
//                  u =  1024 * (id % width) / width;               // u
//                                        v =  1024 * ((id - u) / width) / height;        // v
//                  B.data[0] = u; B.data[1] = v; B.data[2] = 0.0;
//                  ++id;
//                                        u =  1024 * (id % width) / width;               // u
//                                        v =  1024 * ((id - u) / width) / height;        // v
//                  C.data[0] = u; C.data[1] = v; C.data[2] = 0.0;
//                  pointsIterator = pointsIterator + 3;
//              }
//              //printf( "P is %s of the triangle ABC.\n", IsPtInTriangle( P, A, B, C ) ? "inside" : "outside" );
//              bool inTriangle = IsPtInTriangle( P, A, B, C );
//              if( inTriangle == true){
//                  std::cout << "Point is inside triangle" << std::endl;
//              }
//              ++cellIterator;
//          }
//      }
//  }


    //cv::Mat B;
    //std::cout << "Max: " << Max << " Min: " << Min << std::endl;
    //M.convertTo(B, CV_16U);
    //cvtColor(M, M, CV_BGR2Luv);
    cv::imwrite("texture.tiff", outputTexture);

    return 0;
} // end main

