// invertcloud.cpp
// Seth Parker, Aug 2015

#include <fstream>
#include <iostream>

#include <boost/filesystem/path.hpp>
#include "common/io/PointSetIO.h"
#include "common/types/Point.h"
#include "common/types/VolumePkg.h"
#include "common/vc_defines.h"

namespace fs = boost::filesystem;

int main(int argc, char* argv[])
{
    if (argc < 3) {
        std::cout << "Usage: vc_invertCloud volpkg [input].vcps [output].vcps"
                  << std::endl;
        exit(-1);
    }

    VolumePkg vpkg = VolumePkg(argv[1]);
    fs::path input_path = argv[2];
    fs::path output_path = argv[3];

    if (vpkg.getVersion() < 2) {
        std::cerr << "ERROR: Volume package is version " << vpkg.getVersion()
                  << " but this program requires a version >= 2" << std::endl;
        exit(EXIT_FAILURE);
    }

    std::cout << input_path << std::endl;

    // Load the cloud
    volcart::OrderedPointSet<volcart::Point3d> input;
    input = volcart::PointSetIO<volcart::Point3d>::ReadOrderedPointSet(
        input_path.string());

    for (auto pt : input) {
        if (pt[2] != -1)
            pt[2] = vpkg.getNumberOfSlices() - 1 - pt[2];
    }

    volcart::PointSetIO<volcart::Point3d>::WriteOrderedPointSet(
        output_path.string(), input);

    return 0;
}  // end main
