#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <opencv2/core.hpp>

#include "vc/app_support/ProgressIndicator.hpp"
#include "vc/core/io/PointSetIO.hpp"
#include "vc/core/types/PointSet.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/util/Logging.hpp"
#include "vc/segmentation/ComputeVolumetricMask.hpp"

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;
namespace vcs = volcart::segmentation;

void WriteMaskImage(
    int idx, size_t pad, const fs::path& dir, const cv::Mat& img);

int main(int argc, char* argv[])
{
    ///// Parse the command line options /////
    // All command line options
    // clang-format off
    po::options_description all("Usage");
    all.add_options()
        ("help,h", "Show this message")
        ("volpkg,v", po::value<std::string>()->required(), "VolumePkg path")
        ("volume", po::value<std::string>(), "Volume to use for texturing. "
            "Default: The first volume in the volume package.")
        ("input-pts,i", po::value<std::string>()->required(),
            "Path to an input point set representing a segmentation")
        ("output-pts,o", po::value<std::string>()->required(),
         "Path to a directory to store the output point mask");

    // TFF options
    po::options_description tffOptions("Thinned Flood Fill Segmentation Options");
    tffOptions.add_options()
        ("low-thresh,l", po::value<uint16_t>()->default_value(14135),
             "Low threshold for the bounded flood-fill component [0-255]")
        ("high-thresh,t", po::value<uint16_t>()->default_value(65535),
             "High threshold for the bounded flood-fill component [0-255]")
        ("enable-closing", po::value<bool>()->default_value(true),
             "If enabled, perform morphological mask closing.")
        ("closing-kernel-size,k", po::value<int>()->default_value(5),
             "Size of the kernel used for closing")
        ("max-seed-radius", po::value<size_t>(),
            "Max radius a seed point can have when measuring the thickness of the page.")
        ("measure-vert", "Measure the thickness of the page by going vertically (+/- y) "
            "from each seed point (measures horizontally by default)");
    all.add(tffOptions);
    // clang-format on

    // parsed will hold the values of all parsed options as a Map
    po::variables_map parsed;
    po::store(po::command_line_parser(argc, argv).options(all).run(), parsed);

    // Show the help message
    if (parsed.count("help") || argc < 2) {
        std::cout << all << std::endl;
        return EXIT_SUCCESS;
    }

    // Warn of missing options
    try {
        po::notify(parsed);
    } catch (po::error& e) {
        vc::Logger()->error(e.what());
        return EXIT_FAILURE;
    }

    // Get the parsed options
    fs::path volpkgPath = parsed["volpkg"].as<std::string>();

    ///// Load the volume package /////
    auto vpkg = vc::VolumePkg::New(volpkgPath);

    // Load the Volume
    vc::Volume::Pointer volume;
    try {
        if (parsed.count("volume") > 0) {
            volume = vpkg->volume(parsed["volume"].as<std::string>());
        } else {
            volume = vpkg->volume();
        }
    } catch (const std::exception& e) {
        std::cerr << "Cannot load volume. ";
        std::cerr << "Please check that the Volume Package has volumes and "
                     "that the volume ID is correct."
                  << std::endl;
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // Get the input and output paths
    fs::path ptsPath = parsed["input-pts"].as<std::string>();
    fs::path outPath = parsed["output-pts"].as<std::string>();

    // Read the segmentation pointset
    vc::Logger()->info("Reading segmentation");
    auto segmentation = vc::PointSetIO<cv::Vec3d>::ReadPointSet(ptsPath);

    // Setup the mask generator
    vcs::ComputeVolumetricMask maskGen;
    maskGen.setPointSet(segmentation);
    maskGen.setVolume(volume);
    maskGen.setLowThreshold(parsed["low-thresh"].as<uint16_t>());
    maskGen.setHighThreshold(parsed["high-thresh"].as<uint16_t>());
    maskGen.setEnableClosing(parsed["enable-closing"].as<bool>());
    maskGen.setClosingKernelSize(parsed["closing-kernel-size"].as<int>());
    if (parsed.count("max-seed-radius") > 0) {
        auto r = parsed["max-seed-radius"].as<size_t>();
        maskGen.setMaxRadius(r);
    }
    maskGen.setMeasureVertical(parsed.count("measure-vert") > 0);

    // Setup progress reporting
    vc::ReportProgress(maskGen, "Generating mask");

    // Compute the mask
    auto mask = maskGen.compute();

    // Save the mask
    vc::Logger()->info("Saving mask");
    vc::PointSet<cv::Vec3i> maskPts;
    maskPts.append(mask->as_vector());
    vc::PointSetIO<cv::Vec3i>::WritePointSet(outPath, maskPts);
}