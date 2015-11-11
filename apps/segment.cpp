#include <iostream>

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/console/parse.h>
#include <pcl/filters/conditional_removal.h>

#include "volumepkg.h"
#include "localResliceParticleSim/localResliceParticleSim.h"
//#include "structureTensorParticleSim/structureTensorParticleSim.h"

// File paths
std::string segID = "";
std::string volpkgLocation = "";
std::string outputName = "";

// Options
int threshold = 1;
int startIndex = -1;
int endIndex = -1;

int main(int argc, char *argv[]) {
    std::cout << "vc_segment" << std::endl;
    if (argc < 5) {
        std::cerr << "Usage:" << std::endl;
        std::cerr << argv[0] <<
        " --startIndex [Z-Index #] --endIndex [value]} --seg [Seg ID #] --volpkg [volpkgpath] --output [outputname]" <<
        std::endl;
        exit(EXIT_FAILURE);
    }

    // Option parsing

    // NOTE: Distance thresholding causes problems for resumable segmentations
    //       because you currently need to know the previous step distance.
    //       Removing until the pipeline is stronger.
    // pcl::console::parse_argument (argc, argv, "--threshold", threshold);
    // if (threshold == -1) {
    //   std::cout << "No Distance Threshold value given, defaulting to 1" << std::endl;
    //   threshold = 1;
    // }

    pcl::console::parse_argument(argc, argv, "--startIndex", startIndex);

    pcl::console::parse_argument(argc, argv, "--endIndex", endIndex);

    pcl::console::parse_argument(argc, argv, "--seg", segID);

    pcl::console::parse_argument(argc, argv, "--volpkg", volpkgLocation);

    pcl::console::parse_argument(argc, argv, "--output", outputName);
    if (volpkgLocation == "") {
        std::cerr << "ERROR: Incorrect/missing volpkg location!" << std::endl;
        exit(EXIT_FAILURE);
    }
    if (outputName.empty()) {
        std::cerr << "ERROR: Missing output filename" << std::endl;
        exit(EXIT_FAILURE);
    }

    // Load volume package
    VolumePkg volpkg(volpkgLocation);
    // To-Do: Check to make sure the Seg ID is actually in the volpkg
    if (segID == "") {
        std::cerr << "ERROR: Incorrect/missing segmentation ID!" << std::endl;
        exit(EXIT_FAILURE);
    }
    if (volpkg.getVersion() < 2.0) {
        std::cerr << "ERROR: Volume package is version " << volpkg.getVersion() <<
        " but this program requires a version >= 2.0." << std::endl;
        exit(EXIT_FAILURE);
    }
    volpkg.setActiveSegmentation(segID);

    // Setup
    // Run segmentation using path as our starting points
    volcart::segmentation::LocalResliceSegmentation segmentation(volpkg);
    auto segment = segmentation.segmentLayer(3.0, startIndex, endIndex);
	//auto segment = volcart::segmentation::structureTensorParticleSim(volpkg.openCloud(), volpkg, 0.5, 1, endIndex);
    std::cout << "done with segmentation" << std::endl;

    // Save point cloud and mesh
    //volpkg.saveCloud(*masterCloud);
    std::cout << "Saving mutableCloud" << std::endl;
    pcl::io::savePCDFileBinaryCompressed(outputName, segment);
    //volpkg.saveMesh(masterCloud);

    exit(EXIT_SUCCESS);
}
