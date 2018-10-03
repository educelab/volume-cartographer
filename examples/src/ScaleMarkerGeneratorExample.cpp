//
// Created by Kyra Seevers on 10/17/18.
//
#include <iostream>

#include <opencv2/imgcodecs.hpp>

#include "vc/core/types/Color.hpp"
#include "vc/texturing/ScaleMarkerGenerator.hpp"

namespace vc = volcart;
namespace vct = volcart::texturing;

using ScaleGen = vct::ScaleMarkerGenerator;

int main(int argc, char* argv[])
{
    // Check args
    if (argc != 5 && argc != 6) {
        std::cerr << "Usage: " << argv[0]
                  << " [texture] [texture pixel size (um)] [type] [color opt | "
                     "reference image] ([ref. image pixel size (um)])\n";
        std::cerr << "\tType Options: 0 = Metric, 1 = Imperial, 2 = Reference "
                     "Image\n";
        std::cerr << "\tColor Options: 0 = White, 1 = Red, 2 = Green\n";
        return EXIT_FAILURE;
    }

    // Get the texture image and common parameters
    auto inputImg = cv::imread(argv[1], cv::IMREAD_UNCHANGED);
    if (inputImg.empty()) {
        std::cout << "Input image did not load/is empty!" << std::endl;
        return EXIT_FAILURE;
    }
    auto inputPixSize = std::stod(argv[2]);
    auto scaleType = static_cast<ScaleGen::Type>(std::stoi(argv[3]));

    // Setup the scale generator
    ScaleGen scaleGen;
    scaleGen.setInputImage(inputImg);
    scaleGen.setInputImagePixelSize(inputPixSize);
    scaleGen.setScaleType(scaleType);

    // Reference image opt
    if (scaleType == ScaleGen::Type::ReferenceImage) {
        // Num. arguments checking
        if (argc != 6) {
            std::cerr << "Incorrect number of parameters for scale type "
                         "reference image.\n";
            return EXIT_FAILURE;
        }

        // Get the reference image and ref. image pixel size
        auto refImg = cv::imread(argv[4], cv::IMREAD_UNCHANGED);
        if (refImg.empty()) {
            std::cout << "Reference image did not load/is empty!" << std::endl;
            return EXIT_FAILURE;
        }
        auto refPixSize = std::stoi(argv[5]);
        scaleGen.setReferenceImage(refImg);
        scaleGen.setReferenceImagePixelSize(refPixSize);
    }

    // Scale bar opt
    else {
        auto colorOpt = std::stoi(argv[4]);
        switch (colorOpt) {
            case 0:
                scaleGen.setScaleBarColor(vc::color::WHITE);
                break;
            case 1:
                scaleGen.setScaleBarColor(vc::color::RED);
                break;
            case 2:
                scaleGen.setScaleBarColor(vc::color::GREEN);
                break;
            default:
                std::cerr << "Unrecognized color option. Using default.\n";
                scaleGen.setScaleBarColor(vc::color::WHITE);
                break;
        }
    }

    // Compute scale images
    std::cout << "Adding scale markers..." << std::endl;
    auto result = scaleGen.compute();
    if (result.empty()) {
        std::cerr << "Generated image is empty" << std::endl;
        return EXIT_FAILURE;
    }

    // Write the output
    std::cout << "Writing image with scale markers..." << std::endl;
    cv::imwrite("texture_with_scale.png", result);
}
