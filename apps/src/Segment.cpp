#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include "vc/core/types/VolumePkg.hpp"
#include "vc/external/GetMemorySize.hpp"
#include "vc/meshing/OrderedPointSetMesher.hpp"
#include "vc/segmentation/LocalResliceParticleSim.hpp"
#include "vc/segmentation/StructureTensorParticleSim.hpp"

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;
namespace vs = vc::segmentation;

// Volpkg version required by this app
static constexpr int VOLPKG_SUPPORTED_VERSION = 5;

// Default values for global options
static const int kDefaultStep = 1;

// Default values for STPS options
static const double kDefaultGravity = 0.5;

// Default values for LRPS options
static const int kDefaultStartIndex = -1;
static const int kDefaultNumIters = 15;
static const double kDefaultAlpha = 1.0 / 3.0;
static const double kDefaultK1 = 0.5;
static const double kDefaultK2 = 0.5;
static const double kDefaultBeta = 1.0 / 3.0;
static const double kDefaultDelta = 1.0 / 3.0;
static const int kDefaultPeakDistanceWeight = 50;
static const bool kDefaultConsiderPrevious = false;
static constexpr int kDefaultResliceSize = 32;

enum class Algorithm { STPS, LRPS };

int main(int argc, char* argv[])
{
    // Set up options
    // clang-format off
    po::options_description required("Required arguments");
    required.add_options()
        ("help,h", "Show this message")
        ("volpkg,v", po::value<std::string>()->required(), "VolumePkg path")
        ("seg,s", po::value<std::string>()->required(), "Segmentation ID")
        ("method,m", po::value<std::string>()->required(),
            "Segmentation method: LRPS")
        ("volume", po::value<std::string>(),
            "Volume to use for texturing. Default: Segmentation's associated "
            "volume or the first volume in the volume package.")
        ("start-index", po::value<int>()->default_value(kDefaultStartIndex),
            "Starting slice index. Default to highest z-index in path")
        ("end-index", po::value<int>(),
            "Ending slice index. Mutually exclusive with 'stride'")
        ("stride", po::value<int>(),
            "Number of slices to propagate through relative to the starting slice index. "
            "Mutually exclusive with 'end-index'")
        ("step-size", po::value<int>()->default_value(kDefaultStep),
            "Z distance travelled per iteration");


    // STPS options
    po::options_description stpsOptions("Structure Tensor Particle Sim Options");
    stpsOptions.add_options()
        ("gravity-scale", po::value<double>()->default_value(kDefaultGravity),
            "Gravity scale");

    // LRPS options
    po::options_description lrpsOptions("Local Reslice Particle Sim Options");
    lrpsOptions.add_options()
        ("num-iters,n", po::value<int>()->default_value(kDefaultNumIters),
            "Number of optimization iterations")
        ("reslice-size,r", po::value<int>()->default_value(kDefaultResliceSize),
         "Size of reslice window")
        ("alpha,a", po::value<double>()->default_value(kDefaultAlpha),
            "Coefficient for internal energy metric")
        ("k1", po::value<double>()->default_value(kDefaultK1),
            "Coefficient for first derivative term in internal energy metric")
        ("k2", po::value<double>()->default_value(kDefaultK2),
            "Coefficient for second derivative term in internal energy metric")
        ("beta,b", po::value<double>()->default_value(kDefaultBeta),
            "Coefficient for curve tension energy metric")
        ("delta,d", po::value<double>()->default_value(kDefaultDelta),
            "Coefficient for curve curvature energy metric")
        ("distance-weight",
            po::value<int>()->default_value(kDefaultPeakDistanceWeight),
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
    po::variables_map parsed;
    po::store(po::parse_command_line(argc, argv, all), parsed);

    // Display help
    if (argc == 1 || parsed.count("help")) {
        std::cout << all << std::endl;
        std::exit(1);
    }

    // Check mutually exclusive arguments
    if (!(parsed.count("start-index") || parsed.count("stride"))) {
        std::cerr << "[error]: must specify one of [stride, start-index]"
                  << std::endl;
        std::exit(1);
    }
    if (parsed.count("end-index") && parsed.count("stride")) {
        std::cerr << "[error]: 'end-index' and 'stride' are mutually exclusive"
                  << std::endl;
        std::exit(1);
    }

    // Warn of missing options
    try {
        po::notify(parsed);
    } catch (po::error& e) {
        std::cerr << "[error]: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    Algorithm alg;
    auto methodStr = parsed["method"].as<std::string>();
    std::string lower;
    std::transform(
        std::begin(methodStr), std::end(methodStr), std::back_inserter(lower),
        ::tolower);
    std::cout << "Segmentation method: " << lower << std::endl;
    if (lower == "lrps") {
        alg = Algorithm::LRPS;
    } else {
        std::cerr << "[error]: Unknown algorithm type. Must be one of ['LRPS']"
                  << std::endl;
        std::exit(1);
    }

    ///// Load the volume package /////
    fs::path volpkgPath = parsed["volpkg"].as<std::string>();
    vc::VolumePkg vpkg(volpkgPath);
    if (vpkg.version() != VOLPKG_SUPPORTED_VERSION) {
        std::cerr << "ERROR: Volume package is version " << vpkg.version()
                  << " but this program requires version "
                  << VOLPKG_SUPPORTED_VERSION << "." << std::endl;
        return EXIT_FAILURE;
    }

    ///// Load the segmentation /////
    vc::Segmentation::Pointer seg;
    auto segID = parsed["seg"].as<std::string>();
    try {
        seg = vpkg.segmentation(segID);
    } catch (const std::exception& e) {
        std::cerr << "Cannot load segmentation. ";
        std::cerr << "Please check the provided ID: " << segID << std::endl;
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    ///// Load the Volume /////
    vc::Volume::Pointer volume;
    vc::Volume::Identifier volID;

    if (parsed.count("volume")) {
        volID = parsed["volume"].as<std::string>();
    } else if (seg->hasVolumeID()) {
        volID = seg->getVolumeID();
    }

    try {
        if (!volID.empty()) {
            volume = vpkg.volume(volID);
        } else {
            volume = vpkg.volume();
        }
    } catch (const std::exception& e) {
        std::cerr << "Cannot load volume. ";
        std::cerr << "Please check that the Volume Package has volumes and "
                     "that the volume ID is correct. "
                  << volID << std::endl;
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    double cacheBytes = 0.75 * SystemMemorySize();
    volume->setCacheMemoryInBytes(static_cast<size_t>(cacheBytes));

    // Setup
    // Cache arguments
    auto startIndex = parsed["start-index"].as<int>();
    auto step = parsed["step-size"].as<int>();

    // Load the segmentation
    auto masterCloud = seg->getPointSet();

    // Get some info about the cloud, including chain length and z-index's
    // represented by seg.
    auto chainLength = masterCloud.width();
    auto minIndex = static_cast<int>(floor(masterCloud.front()[2]));
    auto maxIndex = static_cast<int>(floor(masterCloud.max()[2]));

    // If no start index is given, our starting path is all of the points
    // already on the largest slice index
    if (startIndex == -1) {
        startIndex = maxIndex;
        std::cout << "No starting index given, defaulting to Highest-Z: "
                  << startIndex << std::endl;
    }

    // Figure out endIndex using either start-index or stride
    int endIndex =
        (parsed.count("end-index") ? parsed["end-index"].as<int>()
                                   : startIndex + parsed["stride"].as<int>());

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
    vc::OrderedPointSet<cv::Vec3d> immutableCloud;
    size_t pathInCloudIndex = startIndex - minIndex;
    if (startIndex > minIndex) {
        immutableCloud = masterCloud.copyRows(0, pathInCloudIndex);
    } else {
        immutableCloud.setWidth(masterCloud.width());
    }

    // Get the starting path pts.
    auto segPath = masterCloud.getRow(pathInCloudIndex);

    // Filter -1 points
    segPath.erase(
        std::remove_if(
            std::begin(segPath), std::end(segPath),
            [](auto e) { return e[2] == -1; }),
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
    vc::OrderedPointSet<cv::Vec3d> mutableCloud;
    if (alg == Algorithm::LRPS) {
        // Run segmentation using path as our starting points
        vs::LocalResliceSegmentation segmenter;
        segmenter.setChain(segPath);
        segmenter.setVolume(volume);
        segmenter.setMaterialThickness(vpkg.materialThickness());
        segmenter.setTargetZIndex(endIndex);
        segmenter.setStepSize(step);
        segmenter.setOptimizationIterations(parsed["num-iters"].as<int>());
        segmenter.setResliceSize(parsed["reslice-size"].as<int>());
        segmenter.setAlpha(parsed["alpha"].as<double>());
        segmenter.setK1(parsed["k1"].as<double>());
        segmenter.setK2(parsed["k2"].as<double>());
        segmenter.setBeta(parsed["beta"].as<double>());
        segmenter.setDelta(parsed["delta"].as<double>());
        segmenter.setDistanceWeightFactor(parsed["distance-weight"].as<int>());
        segmenter.setConsiderPrevious(parsed["consider-previous"].as<bool>());
        segmenter.setVisualize(parsed.count("visualize") > 0);
        segmenter.setDumpVis(parsed.count("dump-vis") > 0);
        mutableCloud = segmenter.compute();
    }

    // Update the master cloud with the points we saved and concat the new
    // points into the space
    immutableCloud.append(mutableCloud);

    // Save point cloud and mesh
    seg->setPointSet(immutableCloud);
}
