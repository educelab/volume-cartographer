// main.cpp
// Abigail Coleman Feb. 2015

#include <iostream>
#include <fstream>

#include <opencv2/opencv.hpp>

#include "volumepkg.h"
#include "vc_defines.h"
#include "io/ply2itk.h"
#include "io/objWriter.h"

#include "meshUtils.h"
#include "texturingUtils.h"

#include <itkRGBPixel.h>

#include "UPointMapping.h"

int main(int argc, char* argv[])
{
    if ( argc < 6 ) {
        std::cout << "Usage: vc_texture2 volpkg seg-id radius texture-method sample-direction" << std::endl;
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
    if ( vpkg.getVersion() < 2.0) {
        std::cerr << "ERROR: Volume package is version " << vpkg.getVersion() << " but this program requires a version >= 2.0."  << std::endl;
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
    
    // declare pointer to new Mesh object
    VC_MeshType::Pointer  mesh = VC_MeshType::New();

    int meshWidth = -1;
    int meshHeight = -1;

    // try to convert the ply to an ITK mesh
    if (!volcart::io::ply2itkmesh(meshName, mesh, meshWidth, meshHeight)){
        exit( -1 );
    };

    // Matrix to store the output texture
    int textureW = meshWidth;
    int textureH = meshHeight;
    cv::Mat outputTexture = cv::Mat::zeros( textureH, textureW, CV_16UC1 );

    // pointID == point's position in 1D list of points
    // [meshX, meshY] == point's position if list was a 2D matrix
    // [u, v] == point's position in the output matrix
    unsigned long pointID, meshX, meshY;
    double u, v;
    
    // Load the slices from the volumepkg
    std::vector< cv::Mat > aImgVol;

    /*  This function is a hack to avoid a refactoring the texturing
        methods. See Issue #12 for more details. */
    // Setup
    int meshLowIndex = (int) mesh->GetPoint(0)[2];
    int meshHighIndex = meshLowIndex + meshHeight;
    int aNumSlices = vpkg.getNumberOfSlices();

    int bufferLowIndex = meshLowIndex - (int) radius;
    if (bufferLowIndex < 0) bufferLowIndex = 0;

    int bufferHighIndex = meshHighIndex + (int) radius;
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

    // Generate per point uv coordinates specifically for the OBJ
    VC_UVMap uvMap;
    std::cerr << "Calculating UV coordinates for final mesh..." << std::endl;
    for ( VC_PointsInMeshIterator point = mesh->GetPoints()->Begin(); point != mesh->GetPoints()->End(); ++point ) {
        pointID = point.Index();

        // Calculate the point's [meshX, meshY] position based on its pointID
        meshX = pointID % meshWidth;
        meshY = (pointID - meshX) / meshWidth;

        // Calculate the point's UV position
        u =  (double) meshX / (double) meshWidth;
        v =  (double) meshY / (double) meshHeight;

        // OBJ UV coordinates start in the bottom-left of an image, but we're calculating them from the top-left
        // Subtract v from 1.0 to account for this
        cv::Vec2d uv( u, 1.0 - v );

        // Add the uv coordinates into our map at the point index specified
        uvMap.insert( {pointID, uv} );
    }

    // Remesh using ACVD
    int numOfSamples = 3000;
    float gradation = 0;

    mesh = resample(mesh, numOfSamples, gradation);


    // Initialize iterators
    VC_CellIterator  cellIterator = mesh->GetCells()->Begin();
    VC_CellIterator  cellEnd      = mesh->GetCells()->End();
    VC_CellType *    cell;
    VC_PointsInCellIterator pointsIterator;

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

            VC_PointType p = mesh->GetPoint(pointID);
            VC_PixelType normal;
            mesh->GetPointData( pointID, &normal );

            // Calculate the point's [meshX, meshY] position based on its pointID
            meshX = pointID % meshWidth;
            meshY = (pointID - meshX) / meshWidth;

            // Calculate the point's pixel position in the output texture
            u = (double) textureW * (double) meshX / (double) meshWidth;
            v = (double) textureH * (double) meshY / (double) meshHeight;

            // Fill in the output pixel with a value
            // cv::Mat.at uses (row, column)
            double value = textureWithMethod( cv::Vec3f(p[0], p[1], p[2]),
                                              cv::Vec3f(normal[0], normal[1], normal[2]),
                                              aImgVol,
                                              aFilterOption,
                                              radius,
                                              minorRadius,
                                              0.5,
                                              aDirectionOption);
            outputTexture.at < unsigned short > (v, u) = (unsigned short) value;

            pointsIterator += 3; // Only consider top left vertices of triangles
        }

        // Only consider every other triangle
        ++cellIterator;
        ++cellIterator;
    }
    std::cout << std::endl;

    // volcart::io::objWriter objwriter("textured.obj", mesh, uvMap, outputTexture);

    // objwriter.write();

    vpkg.saveTextureData(outputTexture);

    return 0;
} // end main

