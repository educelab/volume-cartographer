#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>

#include <boost/program_options.hpp>

#include "vc/app_support/GeneralOptions.hpp"
#include "vc/app_support/GetMemorySize.hpp"
#include "vc/app_support/ProgressIndicator.hpp"
#include "vc/core/filesystem.hpp"
#include "vc/core/io/PointSetIO.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/util/DateTime.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/core/util/MemorySizeStringParser.hpp"
#include "vc/meshing/OrderedPointSetMesher.hpp"
#include "vc/segmentation/LocalResliceParticleSim.hpp"
#include "vc/segmentation/ThinnedFloodFillSegmentation.hpp"

namespace fs = volcart::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;
namespace vs = vc::segmentation;

// Volpkg version required by this app
static constexpr int VOLPKG_MIN_VERSION = 6;

// Default values for global options
static const double kDefaultStep = 1;

// Default values for LRPS options
static const int kDefaultNumIters = 15;
static const double kDefaultAlpha = 1.0 / 3.0;
static const double kDefaultK1 = 0.5;
static const double kDefaultK2 = 0.5;
static const double kDefaultBeta = 1.0 / 3.0;
static const double kDefaultDelta = 1.0 / 3.0;
static const int kDefaultPeakDistanceWeight = 50;
static const bool kDefaultConsiderPrevious = false;
static constexpr int kDefaultResliceSize = 32;

static int SaveInterval{-1};
static int CurrentIteration{0};

enum class Algorithm { LRPS, TFF };

using PointSet = vs::ThinnedFloodFillSegmentation::PointSet;
using VoxelMask = vs::ThinnedFloodFillSegmentation::VoxelMask;

static void WritePointset(const PointSet& pointset);
static void WriteIntermediatePointset(const PointSet& pointset);
static void WriteMaskPointset(const VoxelMask& pointset);

auto main(int argc, char* argv[]) -> int
{
    // Set up options
    // clang-format off
    po::options_description required("Required arguments");
    required.add_options()
        ("volpkg,v", po::value<std::string>()->required(), "VolumePkg path")
        ("seg,s", po::value<std::string>()->required(), "Segmentation ID")
        ("method,m", po::value<std::string>()->required(),
            "Segmentation method: LRPS, TFF")
        ("volume", po::value<std::string>(),
            "Volume to use for texturing. Default: Segmentation's associated "
            "volume or the first volume in the volume package.")
        ("start-index", po::value<std::size_t>(),
            "Starting slice index. Default to highest z-index in path")
        ("end-index", po::value<std::size_t>(),
            "Ending slice index. Mutually exclusive with 'stride'")
        ("stride", po::value<std::size_t>(),
            "Number of slices to propagate through relative to the starting slice index. "
            "Mutually exclusive with 'end-index'")
        ("step-size", po::value<double>()->default_value(kDefaultStep),
            "Z distance travelled per iteration")
        ("dump-vis", "Write full visualization information to disk as algorithm runs")
            ("verbose","Output debugging information");

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
        ("visualize", "Display curve visualization as algorithm runs");

    // TFF options
    po::options_description tffOptions("Thinned Flood Fill Segmentation Options");
    tffOptions.add_options()
        ("tff-low-thresh,l", po::value<std::uint16_t>()->default_value(14135),
             "Low threshold for the bounded flood-fill component [0-65535]")
        ("tff-high-thresh,t", po::value<std::uint16_t>()->default_value(65535),
             "High threshold for the bounded flood-fill component [0-65535]")
        ("tff-dt-thresh", po::value<float>(),
             "Low threshold for the normalized distance transform [0-1]")
        ("closing-kernel-size,k", po::value<int>()->default_value(5),
             "Size of the kernel used for closing")
        ("spur-length", po::value<std::size_t>()->default_value(6),
            "Spurs smaller than this size will be removed from the skeleton.")
        ("max-seed-radius", po::value<std::size_t>(),
            "Max radius a seed point can have when measuring the thickness of the page.")
        ("measure-vert", "Measure the thickness of the page by going vertically (+/- y) "
            "from each seed point (measures horizontally by default)")
        ("save-interval", po::value<int>(),
            "Save the segmentation after a specified number of slices.")
        ("save-mask","Save the mask created by the segmentation algorithm.");
    // clang-format on
    po::options_description all("Usage");
    all.add(GetGeneralOpts()).add(required).add(lrpsOptions).add(tffOptions);

    // Parse and handle options
    po::variables_map parsed;
    po::store(po::parse_command_line(argc, argv, all), parsed);

    // Display help
    if (argc == 1 || parsed.count("help")) {
        std::cout << all << '\n';
        std::exit(1);
    }

    // Set logging level
    if (parsed.count("verbose") > 0) {
        vc::Logger()->set_level(spdlog::level::debug);
    }

    // Check mutually exclusive arguments
    if (parsed.count("end-index") && parsed.count("stride")) {
        std::cerr << "[error]: 'end-index' and 'stride' are mutually exclusive"
                  << '\n';
        std::exit(1);
    }

    // Warn of missing options
    try {
        po::notify(parsed);
    } catch (po::error& e) {
        std::cerr << "[error]: " << e.what() << '\n';
        return EXIT_FAILURE;
    }

    Algorithm alg;
    auto method = parsed["method"].as<std::string>();
    std::transform(method.begin(), method.end(), method.begin(), ::tolower);
    std::cout << "Segmentation method: " << method << std::endl;
    if (method == "lrps") {
        alg = Algorithm::LRPS;
    } else if (method == "tff") {
        alg = Algorithm::TFF;
    } else {
        std::cerr
            << "[error]: Unknown algorithm type. Must be one of ['LRPS', 'TFF']"
            << '\n';
        std::exit(1);
    }

    ///// Load the volume package /////
    fs::path volpkgPath = parsed["volpkg"].as<std::string>();
    if (not fs::exists(volpkgPath)) {
        vc::Logger()->error("VolPkg does not exist: {}", volpkgPath.string());
        return EXIT_FAILURE;
    }
    vc::VolumePkg vpkg(volpkgPath);
    if (vpkg.version() < VOLPKG_MIN_VERSION) {
        vc::Logger()->error(
            "Volume Package is version {} but this program requires version "
            "{}+. ",
            vpkg.version(), VOLPKG_MIN_VERSION);
        return EXIT_FAILURE;
    }

    ///// Load the segmentation /////
    vc::Segmentation::Pointer seg;
    auto segID = parsed["seg"].as<std::string>();
    try {
        seg = vpkg.segmentation(segID);
    } catch (const std::exception& e) {
        std::cerr << "Cannot load segmentation. ";
        std::cerr << "Please check the provided ID: " << segID << '\n';
        std::cerr << e.what() << '\n';
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
                  << volID << '\n';
        std::cerr << e.what() << '\n';
        return EXIT_FAILURE;
    }

    // Set the cache size
    std::size_t cacheBytes;
    if (parsed.count("cache-memory-limit")) {
        auto cacheSizeOpt = parsed["cache-memory-limit"].as<std::string>();
        cacheBytes = vc::MemorySizeStringParser(cacheSizeOpt);
    } else {
        cacheBytes = SystemMemorySize() / 2;
    }
    volume->setCacheMemoryInBytes(cacheBytes);
    std::cout << "Volume Cache :: ";
    std::cout << "Capacity: " << volume->getCacheCapacity() << " || ";
    std::cout << "Size: " << vc::BytesToMemorySizeString(cacheBytes);
    std::cout << std::endl;

    // Setup
    // Load the segmentation
    auto masterCloud = seg->getPointSet();

    // Get some info about the cloud, including chain length and z-index's
    // represented by seg.
    auto chainLength = masterCloud.width();
    auto minIndex = static_cast<std::size_t>(floor(masterCloud.front()[2]));
    auto maxIndex = static_cast<std::size_t>(floor(masterCloud.max()[2]));

    // Cache arguments
    // If no start index is given, our starting path is all of the points
    // already on the largest slice index
    std::size_t startIndex{0};
    if (parsed.count("start-index") == 0) {
        startIndex = maxIndex;
        std::cout
            << "No starting index given. Defaulting to max Z in point set: "
            << startIndex << std::endl;
    } else {
        startIndex = parsed["start-index"].as<std::size_t>();
    }

    // Step size
    auto step = parsed["step-size"].as<double>();

    // Figure out endIndex using either start-index or stride
    std::size_t endIndex{0};
    if (parsed.count("end-index") > 0) {
        endIndex = parsed["end-index"].as<std::size_t>();
    } else if (parsed.count("stride") > 0) {
        endIndex = startIndex + parsed["stride"].as<std::size_t>();
        endIndex = std::min(endIndex, std::size_t(volume->numSlices() - 1));
    } else {
        endIndex = std::size_t(volume->numSlices() - 1);
        std::cout << "No end index given. Defaulting to max Z in volume: "
                  << endIndex << std::endl;
    }

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
    std::size_t pathInCloudIndex = startIndex - minIndex;
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

    // Progress reporting
    auto enableProgress = parsed["progress"].as<bool>();
    vc::ProgressConfig cfg;
    if (parsed.count("progress-interval") > 0) {
        cfg.interval = vc::DurationFromString(
            parsed["progress-interval"].as<std::string>());
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
        if (enableProgress) {
            vc::ReportProgress(segmenter, "Segmenting", cfg);
        }
        mutableCloud = segmenter.compute();
    }

    else if (alg == Algorithm::TFF) {
        vs::ThinnedFloodFillSegmentation segmenter;
        segmenter.setSeedPoints(segPath);
        segmenter.setVolume(volume);
        segmenter.setIterations(endIndex - startIndex + 1);
        segmenter.setFFLowThreshold(
            parsed["tff-low-thresh"].as<std::uint16_t>());
        segmenter.setFFHighThreshold(
            parsed["tff-high-thresh"].as<std::uint16_t>());
        if (parsed.count("tff-dt-thresh") > 0) {
            auto dtt = parsed["tff-dt-thresh"].as<float>();
            segmenter.setDistanceTransformThreshold(dtt);
        }
        segmenter.setClosingKernelSize(parsed["closing-kernel-size"].as<int>());
        segmenter.setSpurLengthThreshold(
            parsed["spur-length"].as<std::size_t>());
        if (parsed.count("max-seed-radius") > 0) {
            auto r = parsed["max-seed-radius"].as<std::size_t>();
            segmenter.setMaxRadius(r);
        }
        segmenter.setMeasureVertical(parsed.count("measure-vert") > 0);
        segmenter.setDumpVis(parsed.count("dump-vis") > 0);

        if (parsed.count("save-interval") > 0) {
            SaveInterval = parsed["save-interval"].as<int>();
            if (SaveInterval > 0) {
                segmenter.pointsetUpdated.connect(WriteIntermediatePointset);
            }
        }
        if (parsed.count("save-mask") > 0) {
            segmenter.maskUpdated.connect(WriteMaskPointset);
        }
        if (enableProgress) {
            vc::ReportProgress(segmenter, "Segmenting", cfg);
        }
        auto skeleton = segmenter.compute();

        // Regular pointsets aren't fully supported in the main logic yet
        // Write our point set and exit early
        WritePointset(skeleton);
        return 0;
    }

    // Update the master cloud with the points we saved and concat the new
    // points into the space
    immutableCloud.append(mutableCloud);

    // Save point cloud and mesh
    seg->setPointSet(immutableCloud);
}

static void WritePointset(const PointSet& pointset)
{
    vc::PointSetIO<cv::Vec3d>::WritePointSet("pointset.vcps", pointset);
}

static void WriteIntermediatePointset(const PointSet& pointset)
{
    // Save intermediate pointsets if we're doing that
    if (++CurrentIteration % SaveInterval == 0) {
        WritePointset(pointset);
    }
}

static void WriteMaskPointset(const VoxelMask& pointset)
{
    vc::PointSetIO<cv::Vec3i>::WritePointSet("mask_pointset.vcps", pointset);
}
