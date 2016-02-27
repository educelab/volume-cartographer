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
        std::cerr << "    " << argv[0] << " "
                  << "--volpkg /path/to/volume.volpkg --start-index [start] "
                     "--end-index [end] --output /path/to/output.pcd --seg "
                     "seg-id [--step S] "
                     "[--visualize I] [--dump-vis] [--resample-perc F] "
                     "[--num-iters N] [--alpha A] [--beta B]"
                  << std::endl;
        std::exit(1);
    }

    // Required args
    int32_t startIndex = -1;
    pcl::console::parse_argument(argc, argv, "--start-index", startIndex);
    if (startIndex == -1) {
        std::cerr << "[error]: startIndex required\n";
        std::exit(1);
    }
    int32_t endIndex = -1;
    pcl::console::parse_argument(argc, argv, "--end-index", endIndex);
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
    int32_t visIndex = -1;
    if (visualize) {
        visIndex =
            pcl::console::parse_argument(argc, argv, "--visualize", visIndex);
    }
    bool dumpVis = pcl::console::find_switch(argc, argv, "--dump-vis");

    int32_t numIters = 10;
    pcl::console::parse_argument(argc, argv, "--num-iters", numIters);

    int32_t step = 1;
    pcl::console::parse_argument(argc, argv, "--step", step);

    double resamplePerc = 0.40;
    pcl::console::parse_argument(argc, argv, "--resample-perc", resamplePerc);

    double alpha = 1.0;
    pcl::console::parse_argument(argc, argv, "--alpha", alpha);

    double beta = 1.0;
    pcl::console::parse_argument(argc, argv, "--beta", beta);

    double gama = 1.0;
    pcl::console::parse_argument(argc, argv, "--gamma", gama);

    double k1 = 1.0;
    pcl::console::parse_argument(argc, argv, "--k1", k1);

    double k2 = 1.0;
    pcl::console::parse_argument(argc, argv, "--k2", k2);

    int32_t peakDistanceWeight = 50;
    pcl::console::parse_argument(argc, argv, "--peak-weight",
                                 peakDistanceWeight);

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
    auto cloud = segmenter.segmentPath(
        initVoxels, resamplePerc, startIndex, endIndex, numIters, step, alpha,
        beta, gama, k1, k2, peakDistanceWeight, dumpVis, visualize, visIndex);

    // Save to output file
    try {
        pcl::io::savePCDFileBinary(outputPath, cloud);
    } catch (const pcl::IOException& ex) {
        std::cerr << ex.what() << std::endl;
    }
}
