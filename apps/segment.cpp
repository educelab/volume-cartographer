#include <iostream>
#include <boost/program_options.hpp>
#include <pcl/io/pcd_io.h>
#include "volumepkg.h"
#include "localResliceParticleSim/localResliceParticleSim.h"
#include "structureTensorParticleSim/structureTensorParticleSim.h"

namespace po = boost::program_options;
namespace vs = volcart::segmentation;

// Default values for options
static const int32_t kDefaultStep = 1;
static const int32_t kDefaultNumIters = 15;
static const double kDefaultAlpha = 0.5;
static const double kDefaultBeta = 0.5;
static const int32_t kDefaultPeakDistanceWeight = 50;
static const bool kDefaultIncludeMiddle = false;

enum class Algorithm { STPS, LRPS };

int main(int argc, char* argv[])
{
    // Set up options
    // clang-format off
    po::options_description required("Required arguments");
    required.add_options()
        ("help,h", "print help")
        ("start-index", po::value<int32_t>()->required(), "slice start index")
        ("end-index", po::value<int32_t>()->required(), "slice end index")
        ("step", po::value<int32_t>()->default_value(kDefaultStep),
            "slice step count")
        ("volpkg", po::value<std::string>()->required(), "VolumePkg path")
        ("seg-id", po::value<std::string>()->required(), "segmentation ID")
        ("method", po::value<std::string>()->required(),
            "segmentation method; one of 'STPS' or 'LRPS'");

    // STPS options
    po::options_description stpsOptions("Structure Tensor Particle Sim Options");
    stpsOptions.add_options()
        ("gravity-scale", po::value<double>(), "gravity scale")
        ("threshold", po::value<int32_t>(), "threshold");

    // LRPS options
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
            "whether 'going straight down' should be an option")
        ("visualize", po::value<int32_t>(),
            "which particle index to visualize as algorithm runs")
        ("dump-vis", "dump visualization as algorithm progresses");
    // clang-format on
    po::options_description all("All options");
    all.add(required).add(stpsOptions).add(lrpsOptions);

    // Parse and handle options
    po::variables_map opts;
    po::store(po::parse_command_line(argc, argv, all), opts);
    if (argc == 1 || opts.count("help")) {
        std::cout << all << std::endl;
        std::exit(1);
    }
    if (opts["method"].as<std::string>() == "STPS" &&
        opts["step"].as<int32_t>() > 1) {
        std::cerr << "[error]: STPS algorithm can only handle stepsize of 1"
                  << std::endl;
        std::exit(1);
    }
    Algorithm alg;
    auto methodStr = opts["method"].as<std::string>();
    std::cout << methodStr << std::endl;
    std::string lower;
    std::transform(std::begin(methodStr), std::end(methodStr),
                   std::back_inserter(lower), ::tolower);
    std::cout << "method: " << lower << std::endl;
    if (lower == "stps") {
        alg = Algorithm::STPS;
    } else if (lower == "lrps") {
        alg = Algorithm::LRPS;
    } else {
        std::cerr << "[error]: Unknown algorithm type. Must be one of ['LRPS', "
                     "'STPS']"
                  << std::endl;
        std::exit(1);
    }

    VolumePkg volpkg(opts["volpkg"].as<std::string>());
    volpkg.setActiveSegmentation(opts["seg-id"].as<std::string>());
    if (volpkg.getVersion() < 2.0) {
        std::cerr << "[error]: Volume package is version "
                  << volpkg.getVersion()
                  << " but this program requires a version >= 2.0."
                  << std::endl;
        std::exit(1);
    }

    // Setup
    // Load the activeSegmentation's current cloud
    auto masterCloud = volpkg.openCloud();

    // Get some info about the cloud, including chain length and z-index's
    // represented by seg.
    const int chainLength = masterCloud->width;
    const int iterations = masterCloud->height;
    pcl::PointXYZRGB min_p, max_p;
    pcl::getMinMax3D(*masterCloud, min_p, max_p);
    int minIndex = floor(masterCloud->points[0].z);
    int maxIndex = floor(max_p.z);

    // Cache arguments
    int32_t startIndex = opts["start-index"].as<int32_t>();
    int32_t endIndex = opts["end-index"].as<int32_t>();
    int32_t step = opts["step"].as<int32_t>();

    // Setup the temp clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr immutableCloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segPath(
        new pcl::PointCloud<pcl::PointXYZRGB>);

    // If no start index is given, our starting path is all of the points
    // already on the largest slice index
    if (startIndex == -1) {
        startIndex = maxIndex;
        std::cout << "No starting index given, defaulting to Highest-Z: "
                  << startIndex << std::endl;
    }

    // Prepare our clouds
    // Get the starting path pts.
    // Find our starting row. NOTE: This currently assumes segmentation distance
    // threshold has always been 1
    int pathRow = startIndex - minIndex;
    for (int i = 0; i < chainLength; ++i) {
        pcl::PointXYZRGB temp_pt;
        temp_pt = masterCloud->points[i + (pathRow * chainLength)];
        if (temp_pt.z != -1) {
            segPath->push_back(temp_pt);
        }
    }

    // Starting paths must have the same number of points as the input width to
    // maintain ordering
    if (segPath->width != chainLength) {
        std::cerr << std::endl;
        std::cerr << "[error]: Starting chain length does not match expected "
                     "chain length."
                  << std::endl;
        std::cerr << "           Expected: " << chainLength << std::endl;
        std::cerr << "           Actual: " << segPath->width << std::endl;
        std::cerr << "       Consider using a lower starting index value."
                  << std::endl
                  << std::endl;
        std::exit(1);
    }

    // Get the immutable points, i.e all pts before the starting path row
    for (int i = 0; i < (pathRow * chainLength); ++i) {
        immutableCloud->push_back(masterCloud->points[i]);
    }
    immutableCloud->width = chainLength;
    immutableCloud->height = immutableCloud->points.size() / chainLength;
    immutableCloud->points.resize(immutableCloud->width *
                                  immutableCloud->height);

    // Run the algorithms
    pcl::PointCloud<pcl::PointXYZRGB> mutableCloud;
    if (alg == Algorithm::STPS) {
        double gravityScale = opts["gravity-scale"].as<double>();
        int32_t threshold = opts["threshold"].as<int32_t>();
        mutableCloud = vs::structureTensorParticleSim(
            segPath, volpkg, gravityScale, threshold,
            endIndex - startIndex + 1);
    } else {
        int32_t numIters = opts["num-iters"].as<int32_t>();
        double alpha = opts["alpha"].as<double>();
        double beta = opts["beta"].as<double>();
        int32_t distanceWeight = opts["distance-weight"].as<int32_t>();
        bool includeMiddle = opts["include-middle"].as<bool>();
        bool visualize = opts.count("visualize");
        bool dumpVis = opts.count("dump-vis");

        // Run segmentation using path as our starting points
        volcart::segmentation::LocalResliceSegmentation segmenter(volpkg);
        mutableCloud = segmenter.segmentPath(
            segPath, startIndex, endIndex, numIters, step, alpha, beta,
            distanceWeight, includeMiddle, dumpVis, visualize);
    }

    // Update the master cloud with the points we saved and concat the new
    // points into the space
    *masterCloud = *immutableCloud;
    *masterCloud += mutableCloud;

    // Restore ordering information
    masterCloud->width = chainLength;
    masterCloud->height = masterCloud->points.size() / masterCloud->width;
    masterCloud->points.resize(masterCloud->width * masterCloud->height);

    // Save point cloud and mesh
    volpkg.saveCloud(*masterCloud);
    volpkg.saveMesh(masterCloud);
}
