// layering.cpp
// Abigail Coleman June 2015

#include <iostream>
#include <fstream>
#include <string>
#include <cmath>

#include <opencv2/opencv.hpp>

#include "texturingUtils.h"
#include "volumepkg.h"
#include "plyHelper.h"
#include "meshUtils.h"

#include <itkMesh.h>
#include <itkTriangleCell.h>

int main(int argc, char* argv[])
{
    if ( argc < 6 ) {
        std::cout << "Usage: vc_texture2 volpkg seg-id smoothing-factor sample-direction ";
        std::cout << "number-of-sections sectioning-scale" << std::endl;
        std::cout << "Sample Direction: " << std::endl;
        std::cout << "      0 = Omni" << std::endl;
        std::cout << "      1 = Positive" << std::endl;
        std::cout << "      2 = Negative" << std::endl;
        std::cout << "Sectioning scale optional, defaults to 1" << std::endl;
        exit( -1 );
    }

    VolumePkg vpkg = VolumePkg( argv[ 1 ] );
    std::string segID = argv[ 2 ];
    if (segID == "") {
        std::cerr << "ERROR: Incorrect/missing segmentation ID!" << std::endl;
        exit(EXIT_FAILURE);
    }
    if ( vpkg.getVersion() != 2.0) {
        std::cerr << "ERROR: Volume package version should be version 2" << std::endl;
        exit(EXIT_FAILURE);
    } 
    vpkg.setActiveSegmentation( segID );
    std::string meshName = vpkg.getMeshPath();
    
    double smoothingFactor;
    smoothingFactor = atof( argv[ 3 ] );

    int aSampleDir = atoi( argv[ 4 ] ); // sampleDirection (0=omni, 1=positive, 2=negative)
    EDirectionOption aDirectionOption = ( EDirectionOption )aSampleDir;

    int sections = atoi( argv[ 5 ] );

    // Sectioning scale is 1 by default, otherwise set by user
    double scale = 1;
    if ( argc > 6 )
        scale = atof( argv[ 6 ] );

    typedef itk::Vector< double, 3 >  PixelType;  // A vector to hold the normals along with the points of each vertice in the mesh
    const unsigned int Dimension = 3;   // Need a 3 Dimensional Mesh

    // declare Mesh object using template parameters 
    typedef itk::Mesh< PixelType, Dimension >   MeshType;
    
    // declare pointer to new Mesh object
    MeshType::Pointer  inputMesh = MeshType::New();
    MeshType::Pointer  smoothedMesh = MeshType::New();

    int meshWidth = -1;
    int meshHeight = -1;

    // try to convert the ply to an ITK mesh
    if ( !ply2itkmesh(meshName, inputMesh, meshWidth, meshHeight) ) {
        exit( -1 );
    };

    // Define iterators
    typedef CellType::PointIdIterator     PointsIterator;
    typedef MeshType::CellsContainer::Iterator  CellIterator;

    // Matrices to store the output textures
    int textureW = meshWidth;
    int textureH = meshHeight;
    // Essential data structure that will be saved as images after sectioning
    // Each matrix represent a section
    cv::Mat *outputTextures = new cv::Mat[ sections ];

    // Allocate each output texture to exact width and height
    for ( int i = 0; i < sections; ++i) {
        outputTextures[ i ] = cv::Mat::zeros( textureH, textureW, CV_16UC1 );
    }

    // pointID == point's position in 1D list of points
    // [meshX, meshY] == point's position if list was a 2D matrix
    // [u, v] == point's position in the output matrix
    unsigned long pointID, meshX, meshY;
    double u, v;

    // Get range (thickness) of material
    double range = vpkg.getMaterialThickness();
    range = range / vpkg.getVoxelSize(); // convert range from microns to pixels
    
    // Load the slices from the volumepkg
    std::vector< cv::Mat > aImgVol;

    /*  This function is a hack to avoid a refactoring the texturing
        methods. See Issue #12 for more details. */
    // Setup
    int meshLowIndex = (int) inputMesh->GetPoint(0)[0];
    int meshHighIndex = meshLowIndex + meshHeight;
    int aNumSlices = vpkg.getNumberOfSlices();

    int bufferLowIndex = meshLowIndex - (int) range;
    if (bufferLowIndex < 0) bufferLowIndex = 0;

    int bufferHighIndex = meshHighIndex + (int) range;
    if (bufferHighIndex >= vpkg.getNumberOfSlices()) bufferHighIndex = vpkg.getNumberOfSlices();

    // Slices must be loaded into aImgVol at the correct index: slice 005 == aImgVol[5]
    // To avoid loading the whole volume, pad the beginning indices with 1x1 null mats
    cv::Mat nullMat = cv::Mat::zeros(1, 1, CV_16U);
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

    // smooth normals if smoothing factor is not 0
    if ( smoothingFactor > 0 ) {
        smoothedMesh = smoothNormals( inputMesh, smoothingFactor);
    }
    else
        smoothedMesh = inputMesh;

    // Initialize iterators
    CellIterator  cellIterator = smoothedMesh->GetCells()->Begin();
    CellIterator  cellEnd      = smoothedMesh->GetCells()->End();
    CellType * cell;
    PointsIterator pointsIterator; 

    // Iterate over all of the cells to lay out the faces in the output texture
    while( cellIterator != cellEnd )
    {
        // Link the pointer to our current cell
        cell = cellIterator.Value();
        
        std::cout << "Texturing face " << cellIterator.Index() << "/" << cellEnd.Index() << "\r" << std::flush;

        // Iterate over the vertices of the current cell
        pointsIterator = cell->PointIdsBegin();
        while( pointsIterator != cell->PointIdsEnd())
        {
            pointID = *pointsIterator;

            MeshType::PointType p = smoothedMesh->GetPoint(pointID);
            MeshType::PixelType normal;
            smoothedMesh->GetPointData( pointID, &normal );
            
            // Calculate the point's [meshX, meshY] position based on its pointID
            meshX = pointID % meshWidth;
            meshY = (pointID - meshX) / meshWidth;

            // Calculate the point's pixel position in the output texture
            u =  (double) textureW * (double) meshX / (double) meshWidth;
            v =  (double) textureH * (double) meshY / (double) meshHeight;

            // pointer to array that holds intensity values calculated from texturing
            double *nData = new double[ sections ];

            Sectioning( sections,
                        range,
                        cv::Vec3f(p[0], p[1], p[2]),
                        cv::Vec3f(normal[1], normal[2], normal[0]),
                        aImgVol,
                        smoothingFactor,
                        aDirectionOption,
                        nData);           

            // Fill in the output pixels with the values from Layering
            for ( int i = 0; i < sections; ++i ) {
                // cv::Mat.at uses (row, column)
                outputTextures[ i ].at < unsigned short > (v, u) = (unsigned short) nData[ i ];
            }

            ++pointsIterator;
        }

        ++cellIterator;
    }
    std::cout << std::endl;

    for ( int i = 0; i < sections; ++i ){
        vpkg.saveTextureData( outputTextures[ i ], std::to_string( i ) );
    }

    return 0;
} // end main

