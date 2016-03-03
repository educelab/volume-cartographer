#include <iostream>
#include <boost/program_options.hpp>
#include <pcl/io/pcd_io.h>
#include "volumepkg.h"
#include "localResliceParticleSim/localResliceParticleSim.h"
#include "structureTensorParticleSim/structureTensorParticleSim.h"

namespace po = boost::program_options;
namespace vs = volcart::segmentation;

// Default values for options
static const int32_t kDefaultNumIters = 15;
static const double kDefaultAlpha = 0.5;
static const double kDefaultBeta = 0.5;
static const int32_t kDefaultPeakDistanceWeight = 50;
static const bool kDefaultIncludeMiddle = false;

int main(int argc, char* argv[])
{
    // Set up options
    // clang-format off
    po::options_description required("Required arguments");
    required.add_options()
        ("help,h", "print help")
        ("start-index", po::value<int32_t>()->required(), "slice start index")
        ("end-index", po::value<int32_t>()->required(), "slice end index")
        ("step", po::value<int32_t>()->required(), "slice step count")
        ("volpkg", po::value<std::string>()->required(), "VolumePkg path")
        ("seg-id", po::value<std::string>()->required(), "segmentation ID")
        ("method", po::value<std::string>()->required(),
            "segmentation method; one of 'STPS' or 'LRPS'");

    po::options_description stpsOptions("Structure Tensor Particle Sim Options");
    stpsOptions.add_options()
        ("gravity-scale", po::value<double>(), "gravity scale")
        ("threshold", po::value<int32_t>(), "threshold");

    po::options_description lrpsOptions("Local Reslice Particle Sim Options");
    lrpsOptions.add_options()
        ("num-iters,n", po::value<int32_t>()->default_value(kDefaultNumIters),
            "Number of optimization iterations")
        ("alpha,a", po::value<double>()->default_value(kDefaultAlpha),
            "coefficient for first derivative term")
        ("beta,b", po::value<double>()->default_value(kDefaultBeta),
            "coefficient for second derivative term")
        ("distance-weight,d",
            po::value<int32_t>()->default_value(kDefaultPeakDistanceWeight),
            "weighting for distance vs maxima intensity")
        ("include-middle,i",
            po::value<bool>()->default_value(kDefaultIncludeMiddle),
            "whether 'going straight down' should be an option");
    // clang-format on
    po::options_description all("All options");
    all.add(required).add(stpsOptions).add(lrpsOptions);

    // Parse and handle options
    po::variables_map opts;
    po::store(po::parse_command_line(argc, argv, all), opts);
    if (opts.count("help")) {
        std::cout << all << std::endl;
        std::exit(1);
    }

    VolumePkg volpkg(opts["volpkg"].as<std::string>());
    volpkg.setActiveSegmentation(opts["seg-id"].as<std::string>());
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
    auto cloud = segmenter.segmentPath(initVoxels, resamplePerc, startIndex,
                                       endIndex, numIters, step, alpha, beta,
                                       peakDistanceWeight, shouldIncludeMiddle,
                                       dumpVis, visualize, visIndex);

    // Save to output file
    try {
        pcl::io::savePCDFileBinary(outputPath, cloud);
    } catch (const pcl::IOException& ex) {
        std::cerr << ex.what() << std::endl;
    }
}
