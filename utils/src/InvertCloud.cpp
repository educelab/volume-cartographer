// invertcloud.cpp
// Seth Parker, Aug 2015

#include <fstream>
#include <iostream>

#include <boost/filesystem/path.hpp>
#include <opencv2/core.hpp>

#include "vc/core/io/PointSetIO.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/vc_defines.hpp"

namespace fs = boost::filesystem;

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cout << "Usage: vc_invertCloud volpkg [input].vcps [output].vcps"
                  << std::endl;
        std::exit(-1);
    }

    volcart::VolumePkg vpkg{argv[1]};
    fs::path inputPath = argv[2];
    fs::path outputPath = argv[3];

    if (vpkg.getVersion() < 2) {
        std::cerr << "ERROR: Volume package is version " << vpkg.getVersion()
                  << " but this program requires a version >= 2" << std::endl;
        std::exit(EXIT_FAILURE);
    }

    std::cout << inputPath << std::endl;

    // Load the cloud
    auto input =
        volcart::PointSetIO<cv::Vec3d>::ReadOrderedPointSet(inputPath.string());

    for (auto pt : input) {
        if (pt[2] != -1) {
            pt[2] = vpkg.getNumberOfSlices() - 1 - pt[2];
        }
    }

    volcart::PointSetIO<cv::Vec3d>::WriteOrderedPointSet(
        outputPath.string(), input);

    return 0;
}  // end main
