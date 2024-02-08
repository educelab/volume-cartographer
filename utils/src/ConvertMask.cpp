// vc_convert_mask: Bidirectional conversion between Point Mask (.vcps) and
// Volume Mask (Image sequence)

#include <cstddef>
#include <cstdint>
#include <map>
#include <regex>
#include <sstream>

#include <boost/program_options.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "vc/app_support/ProgressIndicator.hpp"
#include "vc/core/filesystem.hpp"
#include "vc/core/io/FileExtensionFilter.hpp"
#include "vc/core/io/PointSetIO.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/util/FormatStrToRegexStr.hpp"
#include "vc/core/util/Iteration.hpp"
#include "vc/core/util/Logging.hpp"

namespace fs = volcart::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;

void PointMaskToVolumeMask(
    const fs::path& ptsPath,
    const fs::path& outDir,
    const vc::Volume::Pointer& volume);
void WriteMaskImage(
    int idx, std::size_t pad, const fs::path& dir, const cv::Mat& img);

using SliceList = std::map<std::size_t, fs::path>;
void VolumeMaskToPointMask(const fs::path& inPath, const fs::path& outPath);
auto CollectVolumeFiles(const fs::path& fmtPath) -> SliceList;

auto main(int argc, char* argv[]) -> int
{
    ///// Parse the command line options /////
    // All command line options
    // clang-format off
    po::options_description all("Usage");
    all.add_options()
        ("help,h", "Show this message")
        ("input,i", po::value<std::string>()->required(), "Path to the input mask")
        ("output,o", po::value<std::string>()->required(), "Path to the output mask");

    po::options_description ps2vmOpts("PointSet to Volume Mask Options");
    ps2vmOpts.add_options()
        ("volpkg,v", po::value<std::string>(), "VolumePkg path")
        ("volume", po::value<std::string>(), "Volume to use for texturing. "
           "Default: The first volume in the volume package.");
    all.add(ps2vmOpts);
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

    // Get the input and output paths
    fs::path inPath = parsed["input"].as<std::string>();
    fs::path outPath = parsed["output"].as<std::string>();

    if (vc::IsFileType(inPath, {"vcps"})) {
        // Load the volume package
        if (parsed.count("volpkg") == 0) {
            vc::Logger()->error(
                "the option '--volpkg' is required but missing");
            return EXIT_FAILURE;
        }
        fs::path volpkgPath = parsed["volpkg"].as<std::string>();
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
            vc::Logger()->error("Cannot load volume: {}", e.what());
            return EXIT_FAILURE;
        }

        // Setup output directory
        if (not fs::is_directory(outPath)) {
            vc::Logger()->error(
                "Output path is not directory: {}", outPath.string());
            return EXIT_FAILURE;
        }

        // Do conversion
        PointMaskToVolumeMask(inPath, outPath, volume);

    } else if (vc::IsFileType(inPath, {"jpg", "png", "tif", "bmp"})) {
        VolumeMaskToPointMask(inPath, outPath);
    } else {
        vc::Logger()->error("Unknown file format: {}", inPath.string());
        std::exit(EXIT_FAILURE);
    }

    vc::Logger()->info("Done.");
}

void PointMaskToVolumeMask(
    const fs::path& ptsPath,
    const fs::path& outDir,
    const vc::Volume::Pointer& volume)
{
    // Read the points
    vc::Logger()->info("Loading point mask");
    auto ptsRaw = vc::PointSetIO<cv::Vec3i>::ReadPointSet(ptsPath);

    // Sort the points
    vc::Logger()->info("Sorting points");
    auto pts = ptsRaw.as_vector();
    std::sort(pts.begin(), pts.end(), [](const auto& a, const auto& b) {
        return a[2] < b[2];
    });

    // Mask the images
    vc::Logger()->info("Converting mask");
    std::size_t pad = std::to_string(volume->numSlices()).size();
    int sliceNum{pts[0][2]};
    cv::Size sliceSize(volume->sliceWidth(), volume->sliceHeight());
    cv::Mat slice = cv::Mat::zeros(sliceSize, CV_8UC1);
    for (const auto& p : vc::ProgressWrap(pts, "Masking voxels")) {
        // If the slice number changes...
        if (p[2] > sliceNum) {
            // Write mask
            WriteMaskImage(sliceNum, pad, outDir, slice);

            // Set the new slice number and clear the slice
            sliceNum = p[2];
            slice = cv::Mat::zeros(sliceSize, CV_8UC1);
        }

        slice.at<std::uint8_t>(p[1], p[0]) = 255;
    }
    WriteMaskImage(sliceNum, pad, outDir, slice);
}

void WriteMaskImage(
    int idx, std::size_t pad, const fs::path& dir, const cv::Mat& img)
{
    // Construct the file name
    std::ostringstream file;
    file.fill('0');
    file.width(pad);
    file << idx << ".png";
    auto path = dir / file.str();

    // Write the existing image
    cv::imwrite(path.string(), img);
}

void VolumeMaskToPointMask(const fs::path& inPath, const fs::path& outPath)
{
    vc::PointSet<cv::Vec3i> pts;

    // Collect files
    vc::Logger()->info("Collecting file list");
    auto files = CollectVolumeFiles(inPath);
    vc::Logger()->info("Found {} slice images", files.size());

    // Iterate over files
    vc::Logger()->info("Converting mask");
    for (const auto& p : vc::ProgressWrap(files, "Pixels to points")) {
        const auto& z = p.first;
        const auto& fpath = p.second;

        auto img = cv::imread(fpath.string(), cv::IMREAD_GRAYSCALE);
        for (const auto px : vc::range2D(img.rows, img.cols)) {
            const auto& x = px.second;
            const auto& y = px.first;
            if (img.at<std::uint8_t>(y, x) > 0) {
                pts.emplace_back(x, y, z);
            }
        }
    }

    // Write point set
    vc::Logger()->info("Writing point set...");
    vc::PointSetIO<cv::Vec3i>::WritePointSet(outPath, pts);
}

auto CollectVolumeFiles(const fs::path& fmtPath) -> SliceList
{
    SliceList paths;

    // Construct regex
    std::regex filter{vc::FormatStrToRegexStr(fmtPath.filename().string())};

    // Iterate through all files in the parent directory
    fs::directory_iterator subfile(fmtPath.parent_path());
    fs::directory_iterator dirEnd;
    for (; subfile != dirEnd; subfile++) {
        // Skip if not a regular file
        if (!fs::is_regular_file(subfile->path())) {
            continue;
        }

        // Filter by regex
        auto filename = subfile->path().filename().string();
        std::smatch matches;
        if (std::regex_match(filename, matches, filter)) {
            std::size_t idx = std::stoull(matches[1].str());
            paths[idx] = subfile->path();
        }
    }

    return paths;
}