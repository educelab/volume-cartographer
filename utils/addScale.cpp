// addScale - A utility to create
#include <stdio.h>
#include <string>
#include <opencv2/opencv.hpp>

#include "volumepkg.h"

int main( int argc, char *argv[] )
{
    if( argc < 4 )
    {
    fprintf( stderr, "Usage: %s [volpkg] [segid] [scale index]\n", argv[0] );
    fprintf( stderr, "       Scale Indexes:\n" );
    fprintf( stderr, "          0 = pi\n" );
    fprintf( stderr, "          1 = mini\n" );
    fprintf( stderr, "          2 = small\n" );
    fprintf( stderr, "          3 = large\n" );
    return 1;
    }

    // Load our sources from the volpkg
    VolumePkg volpkg = VolumePkg( argv[ 1 ] );
    std::string segID = argv[ 2 ];
    if (segID == "") {
        std::cerr << "ERROR: Incorrect/missing segmentation ID!" << std::endl;
        exit(EXIT_FAILURE);
    }
    volpkg.setActiveSegmentation(segID);

    // Define which scale image we're going to use
    std::string scaleImagePath;
    switch ( atoi(argv[3]) ) {
        case 0:
            scaleImagePath = "vc_scale_pi.tif";
            break;
        case 1:
            scaleImagePath = "vc_scale_mini.tif";
            break;
        case 2:
            scaleImagePath = "vc_scale_sm.tif";
            break;
        case 3:
            scaleImagePath = "vc_scale_lg.tif";
            break;
        default:
            scaleImagePath = "vc_scale_mini.tif";
            break;
    };

    cv::Mat textureImage = volpkg.getTextureData().clone();
    cv::Mat scaleImage = cv::imread( scaleImagePath, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH );

    // Setup the output image
    cv::Mat outImage(textureImage.rows, textureImage.cols, CV_16U);
    textureImage.copyTo(outImage);

    // Resize the scaleSrc image
    cv::Mat resizedScale;
    double scaleVoxelSize = 40.0;
    double scaleFactor = scaleVoxelSize/volpkg.getVoxelSize();
    resize(scaleImage, resizedScale, cv::Size(), scaleFactor, scaleFactor);

    // Overlay Image
    double originX = outImage.cols - resizedScale.cols;
    double originY = outImage.rows - resizedScale.rows;
    resizedScale.copyTo(outImage(cv::Rect(originX, originY, resizedScale.cols, resizedScale.rows)));

    cv::imwrite( "texture_with_scale.tif", outImage );
    
return 0;
}