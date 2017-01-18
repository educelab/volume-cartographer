// addScale - A utility to create
#include <iostream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "core/scales/Scales.hpp"
#include "core/types/VolumePkg.hpp"

int main(int argc, char** argv)
{
    if (argc < 4) {
        std::cerr << "Usage:" << std::endl;
        std::cerr << "  Scale Indices" << std::endl;
        std::cerr << "      0 = 3mm scale bar" << std::endl;
        std::cerr << "      1 = 10mm scale bar" << std::endl;
        std::cerr << "      2 = Greek Pi" << std::endl;
        return 1;
    }

    // Load our sources from the volpkg
    VolumePkg volpkg(argv[1]);
    std::string segID = argv[2];
    if (segID == "") {
        std::cerr << "ERROR: Incorrect/missing segmentation ID!" << std::endl;
        exit(EXIT_FAILURE);
    }
    volpkg.setActiveSegmentation(segID);
    // Get the Texture Image
    cv::Mat textureImage = volpkg.getTextureData().clone();

    // Define which scale image we're going to use
    cv::Mat scaleImage;
    switch (atoi(argv[3])) {
        case 0:
            scaleImage = cv::Mat(23, 82, CV_16U, &g_scaleMicro);
            break;
        case 1:
            scaleImage = cv::Mat(27, 254, CV_16U, &g_scaleSmall);
            break;
        case 2:
            scaleImage = cv::Mat(76, 94, CV_16U, &g_scalePi);
            break;
        default:
            scaleImage = cv::Mat(23, 82, CV_16U, &g_scaleSmall);
            break;
    }
    scaleImage.convertTo(scaleImage, CV_16U, 65355, 0);

    // Setup the output image
    cv::Mat outImage(textureImage.rows, textureImage.cols, CV_16U);
    textureImage.copyTo(outImage);

    // Resize the scale image to match the voxel size of the dataset
    cv::Mat resizedScale;
    double scaleVoxelSize = 40.0;
    double scaleFactor = scaleVoxelSize / volpkg.getVoxelSize();
    resize(scaleImage, resizedScale, cv::Size(), scaleFactor, scaleFactor);
    if (resizedScale.cols > outImage.cols ||
        resizedScale.rows > outImage.rows) {
        std::cerr << "ERROR: The selected scale image is larger than the "
                     "texture and cannot be mapped. Try using scale index 0."
                  << std::endl;
        return EXIT_FAILURE;
    }

    // Overlay Image
    int originX = outImage.cols - resizedScale.cols;
    int originY = outImage.rows - resizedScale.rows;
    if (originX < 0) {
        originX = 0;
    }
    if (originY < 0) {
        originY = 0;
    }

    cv::Mat outROI = outImage(
        cv::Rect(originX, originY, resizedScale.cols, resizedScale.rows));
    cv::add(outROI, resizedScale, outROI);

    cv::imwrite("texture_with_scale.png", outImage);

    return EXIT_SUCCESS;
}
