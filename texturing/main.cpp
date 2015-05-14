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
    if ( argc < 6 ) {
        std::cout << "Usage: vc_texture volpkg seg-id radius texture-method sample-direction" << std::endl;
        std::cout << "Texture methods: " << std::endl;
        std::cout << "      0 = Intersection" << std::endl;
        std::cout << "      1 = Non-Maximum Suppression" << std::endl;
        std::cout << "      2 = Maximum" << std::endl;
        std::cout << "      3 = Minimum" << std::endl;
        std::cout << "      4 = Median w/ Averaging" << std::endl;
        std::cout << "      5 = Median" << std::endl;
        std::cout << "      6 = Mean" << std::endl;
        std::cout << std::endl;
        std::cout << "Sample Direction: " << std::endl;
        std::cout << "      0 = Omni" << std::endl;
        std::cout << "      1 = Positive" << std::endl;
        std::cout << "      2 = Negative" << std::endl;
        exit( -1 );
    }

    VolumePkg vpkg = VolumePkg( argv[ 1 ] );
    std::string segID = argv[ 2 ];
    if (segID == "") {
        std::cerr << "ERROR: Incorrect/missing segmentation ID!" << std::endl;
        exit(EXIT_FAILURE);
    }
    vpkg.setActiveSegmentation(segID);
    std::string meshName = vpkg.getMeshPath();
    double radius, minorRadius;
    radius = atof( argv[ 3 ] );
    if((minorRadius = radius / 3) < 1) minorRadius = 1;

    int aFindBetterTextureMethod = atoi( argv[ 4 ] );
    EFilterOption aFilterOption = ( EFilterOption )aFindBetterTextureMethod;

    int aSampleDir = atoi( argv[ 5 ] ); // sampleDirection (0=omni, 1=positive, 2=negative)
    EDirectionOption aDirectionOption = ( EDirectionOption )aSampleDir;

    typedef itk::Vector< double, 3 >    PixelType;  // A vector to hold the normals along with the points of each vertice in the mesh
    const unsigned int Dimension = 3;   // Need a 3 Dimensional Mesh

    // declare Mesh object using template parameters 
    typedef itk::Mesh< PixelType, Dimension >   MeshType;
    
    // declare pointer to new Mesh object
    MeshType::Pointer  mesh = MeshType::New();

    int meshWidth = -1;
    int meshHeight = -1;

    // try to convert the ply to an ITK mesh
    if (!ply2itkmesh(meshName, mesh, meshWidth, meshHeight)){
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
    cv::Mat outputTexture = cv::Mat::zeros( textureH, textureW, CV_16UC1 );

    // pointID == point's position in 1D list of points
    // [meshX, meshY] == point's position if list was a 2D matrix
    // [u, v] == point's position in the output matrix
    unsigned long pointID, meshX, meshY;
    double u, v;

    double Max = 0, Min = 8000;
        
    // Load the slices from the volumepkg
    std::vector< cv::Mat > aImgVol;

    /*  This function is a hack to avoid a refactoring the texturing
        methods. See Issue #12 for more details. */
    // Setup
    int meshLowIndex = (int) mesh->GetPoint(0)[0];
    int meshHighIndex = meshLowIndex + meshHeight;
    int aNumSlices = vpkg.getNumberOfSlices();

    int bufferLowIndex = meshLowIndex - (int) radius;
    if (bufferLowIndex < 0) bufferLowIndex = 0;

    int bufferHighIndex = meshHighIndex + (int) radius;
    if (bufferHighIndex >= vpkg.getNumberOfSlices()) bufferHighIndex = vpkg.getNumberOfSlices();

    // Load null mats into the vector
    cv::Mat nullMat = cv::Mat::zeros(vpkg.getSliceHeight(), vpkg.getSliceWidth(), CV_16U);
    for ( int i = 0; i < bufferLowIndex; ++i ) {
        std::cout << "\rLoading null buffer slices: " << i + 1 << "/" << bufferLowIndex << std::flush;
        aImgVol.push_back( nullMat.clone() );
    }
    std::cout << std::endl;

    // Load the actual volume into a tempVol with a buffer of nRadius
    for ( int i = bufferLowIndex; i < bufferHighIndex; ++i ) {
        std::cout << "\rLoading real slices: " << i - bufferLowIndex + 1 << "/" << bufferHighIndex - bufferLowIndex << std::flush;
        aImgVol.push_back( vpkg.getSliceData( i ).clone() );
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
        std::cout << "Texturing face " << cellIterator.Index() << "/" << cellEnd.Index() << "\r" << std::flush;

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
                double value = textureWithMethod(cv::Vec3f(p[0], p[1], p[2]),
                                                 cv::Vec3f(normal[1], normal[2], normal[0]),
                                                 aImgVol,
                                                 aFilterOption,
                                                 radius,
                                                 minorRadius,
                                                 0.5,
                                                 aDirectionOption);
                outputTexture.at < unsigned short > (v, u) = (unsigned short) value;

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
    std::cout << std::endl;

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
    vpkg.saveTextureData(outputTexture);

    return 0;
} // end main

