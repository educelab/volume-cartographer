#include <iostream>
#include <boost/program_options.hpp>
#include "common/types/VolumePkg.h"
#include "meshing/OrderedPointSetMesher.h"
#include "segmentation/lrps/localResliceParticleSim.h"
#include "segmentation/stps/structureTensorParticleSim.h"

namespace po = boost::program_options;
namespace vs = volcart::segmentation;

// Volpkg version required by this app
static constexpr int VOLPKG_SUPPORTED_VERSION = 3;

// Default values for global options
static const int32_t kDefaultStep = 1;

// Default values for STPS options
static const double kDefaultGravity = 0.5;

// Default values for LRPS options
static const int32_t kDefaultStartIndex = -1;
static const int32_t kDefaultNumIters = 15;
static const double kDefaultAlpha = 1.0 / 3.0;
static const double kDefaultK1 = 0.5;
static const double kDefaultK2 = 0.5;
static const double kDefaultBeta = 1.0 / 3.0;
static const double kDefaultDelta = 1.0 / 3.0;
static const int32_t kDefaultPeakDistanceWeight = 50;
static const bool kDefaultConsiderPrevious = false;

enum class Algorithm { STPS, LRPS };

int main(int argc, char* argv[])
{
    // Set up options
    // clang-format off
    po::options_description required("Required arguments");
    required.add_options()
        ("help,h", "Show this message")
        ("volpkg,v", po::value<std::string>()->required(), "VolumePkg path")
        ("seg-id,s", po::value<std::string>()->required(), "Segmentation ID")
        ("method,m", po::value<std::string>()->required(),
            "Segmentation method: STPS, LRPS")
        ("start-index", po::value<int32_t>()->default_value(kDefaultStartIndex),
            "Starting slice index. Default to highest z-index in path")
        ("end-index", po::value<int32_t>(),
            "Ending slice index. Mutually exclusive with 'stride'")
        ("stride", po::value<int32_t>(),
            "Number of slices to propagate through relative to the starting slice index. "
            "Mutually exclusive with 'end-index'")
        ("step-size", po::value<int32_t>()->default_value(kDefaultStep),
            "Z distance travelled per iteration");


    // STPS options
    po::options_description stpsOptions("Structure Tensor Particle Sim Options");
    stpsOptions.add_options()
        ("gravity-scale", po::value<double>()->default_value(kDefaultGravity),
            "Gravity scale");

    // LRPS options
    po::options_description lrpsOptions("Local Reslice Particle Sim Options");
    lrpsOptions.add_options()
        ("num-iters,n", po::value<int32_t>()->default_value(kDefaultNumIters),
            "Number of optimization iterations")
        ("alpha,a", po::value<double>()->default_value(kDefaultAlpha),
            "Coefficient for internal energy metric")
        ("k1", po::value<double>()->default_value(kDefaultK1),
            "Coefficient for first derivative term in internal energy metric")
        ("k2", po::value<double>()->default_value(kDefaultK2),
            "Coefficient for second derivative term in internal energy metric")
        ("beta,b", po::value<double>()->default_value(kDefaultBeta),
            "Coefficient for curve tension energy metric")
        ("delta", po::value<double>()->default_value(kDefaultDelta),
            "Coefficient for curve curvature energy metric")
        ("distance-weight,d",
            po::value<int32_t>()->default_value(kDefaultPeakDistanceWeight),
            "Weighting for distance vs maxima intensity")
        ("consider-previous,p",
            po::value<bool>()->default_value(kDefaultConsiderPrevious),
            "Consider propagation of a point's previous XY position as a "
            "candidate when optimizing each iteration")
        ("visualize", "Display curve visualization as algorithm runs")
        ("dump-vis", "Write full visualization information to disk as algorithm runs");
    // clang-format on
    po::options_description all("Usage");
    all.add(required).add(stpsOptions).add(lrpsOptions);

    // Parse and handle options
    po::variables_map opts;
    po::store(po::parse_command_line(argc, argv, all), opts);

    // Display help
    if (argc == 1 || opts.count("help")) {
        std::cout << all << std::endl;
        std::exit(1);
    }

    // Check mutually exclusive arguments
    if (!(opts.count("start-index") || opts.count("stride"))) {
        std::cerr << "[error]: must specify one of [stride, start-index]"
                  << std::endl;
        std::exit(1);
    }
    if (opts.count("end-index") && opts.count("stride")) {
        std::cerr << "[error]: 'end-index' and 'stride' are mutually exclusive"
                  << std::endl;
        std::exit(1);
    }

    // Warn of missing options
    try {
        po::notify(opts);
    } catch (po::error& e) {
        std::cerr << "[error]: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    Algorithm alg;
    auto methodStr = opts["method"].as<std::string>();
    std::string lower;
    std::transform(
        std::begin(methodStr), std::end(methodStr), std::back_inserter(lower),
        ::tolower);
    std::cout << "Segmentation method: " << lower << std::endl;
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
    if (volpkg.getVersion() != VOLPKG_SUPPORTED_VERSION) {
        std::cerr << "[error]: Volume package is version "
                  << volpkg.getVersion()
                  << " but this program requires a version "
                  << std::to_string(VOLPKG_SUPPORTED_VERSION) << "."
                  << std::endl;
        std::exit(1);
    }

    // Setup
    // Cache arguments
    int32_t startIndex = opts["start-index"].as<int32_t>();
    int32_t step = opts["step-size"].as<int32_t>();
    if (alg == Algorithm::STPS && step != 1) {
        std::cerr << "[warning]: STPS algorithm can only handle stepsize of 1. "
                     "Defaulting to 1."
                  << std::endl;
        step = 1;
    }

    // Load the activeSegmentation's current cloud
    auto masterCloud = volpkg.openCloud();

    // Get some info about the cloud, including chain length and z-index's
    // represented by seg.
    auto chainLength = masterCloud.width();
    auto iterations = masterCloud.height();
    int minIndex = floor(masterCloud.front()[2]);
    int maxIndex = floor(masterCloud.max()[2]);

    // If no start index is given, our starting path is all of the points
    // already on the largest slice index
    if (startIndex == -1) {
        startIndex = maxIndex;
        std::cout << "No starting index given, defaulting to Highest-Z: "
                  << startIndex << std::endl;
    }

    // Figure out endIndex using either start-index or stride
    int32_t endIndex =
        (opts.count("end-index") ? opts["end-index"].as<int32_t>()
                                 : startIndex + opts["stride"].as<int32_t>());

    // Sanity check for whether we actually need to run the algorithm
    if (startIndex >= endIndex) {
        std::cerr << "[info]: startIndex(" << startIndex << ") >= endIndex("
                  << endIndex
                  << "), do not need to segment. Consider using --stride "
                     "option instead of manually specifying endIndex"
                  << std::endl;
        std::exit(1);
    }

    // Prepare our clouds
    // Get the upper, immutable cloud
    auto immutableCloud = masterCloud.copyRows(minIndex, startIndex - 1);

    // Get the starting path pts.
    auto segPath = masterCloud.getRow(startIndex);

    // Filter -1 points
    segPath.erase(
        std::remove_if(
            std::begin(segPath), std::end(segPath),
            [](volcart::Point3d e) { return e[2] == -1; }),
        std::end(segPath));

    // Starting paths must have the same number of points as the input width to
    // maintain ordering
    if (segPath.size() != chainLength) {
        std::cerr << std::endl;
        std::cerr << "[error]: Starting chain length does not match expected "
                     "chain length."
                  << std::endl;
        std::cerr << "           Expected: " << chainLength << std::endl;
        std::cerr << "           Actual: " << segPath.size() << std::endl;
        std::cerr << "       Consider using a lower starting index value."
                  << std::endl
                  << std::endl;
        std::exit(1);
    }

    // Run the algorithms
    volcart::OrderedPointSet<volcart::Point3d> mutableCloud;
    if (alg == Algorithm::STPS) {
        double gravityScale = opts["gravity-scale"].as<double>();
        mutableCloud = vs::structureTensorParticleSim(
            segPath, volpkg, gravityScale, step, endIndex - startIndex);
    } else {
        int32_t numIters = opts["num-iters"].as<int32_t>();
        double alpha = opts["alpha"].as<double>();
        double k1 = opts["k1"].as<double>();
        double k2 = opts["k1"].as<double>();
        double beta = opts["beta"].as<double>();
        double delta = opts["delta"].as<double>();
        int32_t distanceWeight = opts["distance-weight"].as<int32_t>();
        bool considerPrevious = opts["consider-previous"].as<bool>();
        bool visualize = opts.count("visualize");
        bool dumpVis = opts.count("dump-vis");

        // Run segmentation using path as our starting points
        vs::LocalResliceSegmentation segmenter(volpkg);
        mutableCloud = segmenter.segmentPath(
            segPath, startIndex, endIndex, numIters, step, alpha, k1, k2, beta,
            delta, distanceWeight, considerPrevious, dumpVis, visualize);
    }

    // Update the master cloud with the points we saved and concat the new
    // points into the space
    immutableCloud.append(mutableCloud);

    // Save point cloud and mesh
    volpkg.saveCloud(immutableCloud);
    volcart::meshing::OrderedPointSetMesher mesher{immutableCloud};
    mesher.compute();
    volpkg.saveMesh(mesher.getOutputMesh());
}
