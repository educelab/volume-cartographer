#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/filters/conditional_removal.h>
#include "volumepkg.h"
#include "localResliceParticleSim/localResliceParticleSim.h"

int main(int argc, char* argv[])
{
    if (argc < 5) {
        std::cerr << "Usage:" << std::endl;
        std::cerr << "    " << argv[0]
                  << "--volpkg /path/to/volume.volpkg --startIndex [start] "
                     "--endIndex [end] --seg [segID] --step [step] --output "
                     "/path/to/output.pcd [--visualize]\n";
        std::exit(1);
    }

    // Required args
    int32_t startIndex = -1;
    pcl::console::parse_argument(argc, argv, "--startIndex", startIndex);
    if (startIndex == -1) {
        std::cerr << "[error]: startIndex required\n";
        std::exit(1);
    }
    int32_t endIndex = -1;
    pcl::console::parse_argument(argc, argv, "--endIndex", endIndex);
    if (endIndex == -1) {
        std::cerr << "[error]: endIndex required\n";
        std::exit(1);
    }

    std::string segID = "";
    pcl::console::parse_argument(argc, argv, "--seg", segID);
    if (segID == "") {
        std::cerr << "[error]: segID required\n";
        std::exit(1);
    }

    std::string volpkgPath = "";
    pcl::console::parse_argument(argc, argv, "--volpkg", volpkgPath);
    if (volpkgPath == "") {
        std::cerr << "[error]: volpkg required\n";
        std::exit(1);
    }

    std::string outputPath = "";
    pcl::console::parse_argument(argc, argv, "--output", outputPath);
    if (outputPath == "") {
        std::cerr << "[error]: output path required\n";
        std::exit(1);
    }

    // Optional args
    bool visualize = pcl::console::find_switch(argc, argv, "--visualize");

    int32_t step = 1;
    pcl::console::parse_argument(argc, argv, "--step", step);

    VolumePkg volpkg(volpkgPath);
    volpkg.setActiveSegmentation(segID);
    if (volpkg.getVersion() < 2.0) {
        std::cerr << "ERROR: Volume package is version " << volpkg.getVersion()
                  << " but this program requires a version >= 2.0."
                  << std::endl;
        std::exit(1);
    }

    // Change cloud into VoxelVec
    std::vector<cv::Vec3d> initVoxels;
    auto path = volpkg.openCloud();
    for (const auto& p : *path) {
        initVoxels.emplace_back(p.x, p.y, p.z);
    }

    // Run segmentation using path as our starting points
    volcart::segmentation::LocalResliceSegmentation segmenter(volpkg);
    auto cloud = segmenter.segmentLayer(visualize, startIndex, endIndex, step);

    // Save to output file
    try {
        pcl::io::savePCDFileBinary(outputPath, cloud);
    } catch (const pcl::IOException& ex) {
        std::cerr << ex.what() << std::endl;
    }
}
