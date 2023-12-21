#include <boost/program_options.hpp>
#include <opencv2/imgproc.hpp>

#include "vc/core/filesystem.hpp"
#include "vc/core/io/TIFFIO.hpp"
#include "vc/core/types/PerPixelMap.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/util/Logging.hpp"

namespace fs = volcart::filesystem;
namespace po = boost::program_options;
namespace vc = volcart;

auto main(int argc, char* argv[]) -> int
{
    ///// Parse the command line options /////
    // All command line options
    // clang-format off
    po::options_description required("General Options");
    required.add_options()
        ("help,h", "Show this message")
        ("volpkg,v", po::value<std::string>(), "VolumePkg path")
        ("volume", po::value<std::string>(), "Volume to use for position "
            "normalization. Default: The first volume in the volume package.")
        ("ppm,p", po::value<std::string>()->required(), "Input PPM file")
        ("output-prefix,o", po::value<std::string>()->required(),
            "Prefix for output files")
        ("raw,r", "Output the raw position and normal values. "
            "By default, values are normalized between [0, 1].")
        ("tspace-normals", "Project normals to tangent space (WIP)");

    po::options_description all("Usage");
    all.add(required);
    // clang-format on

    // Parse the cmd line
    po::variables_map parsed;
    po::store(po::command_line_parser(argc, argv).options(all).run(), parsed);

    // Show the help message
    if (parsed.count("help") > 0 || argc < 2) {
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

    // Are we normalizing?
    auto normalize = parsed.count("raw") == 0;
    // Project normals to tangent space
    const bool tspace =
        parsed.count("raw") == 0 && parsed.count("tspace-normals") > 0;

    // Load the volume
    vc::Volume::Pointer vol;
    cv::Vec3d dims;
    if (normalize) {
        if (parsed.count("volpkg") == 0) {
            vc::Logger()->error(
                "the option '--volpkg' is required but missing");
            return EXIT_FAILURE;
        }
        const fs::path volpkgPath = parsed["volpkg"].as<std::string>();
        auto vpkg = vc::VolumePkg::New(volpkgPath);
        try {
            if (parsed.count("volume") > 0) {
                vol = vpkg->volume(parsed["volume"].as<std::string>());
            } else {
                vol = vpkg->volume();
            }
        } catch (const std::exception& e) {
            vc::Logger()->error(
                "Cannot load volume. Please check that the Volume Package has "
                "volumes and that the volume ID is correct.");
            return EXIT_FAILURE;
        }
        dims = {
            static_cast<double>(vol->sliceWidth()),
            static_cast<double>(vol->sliceHeight()),
            static_cast<double>(vol->numSlices())};
    }

    // Get input file
    vc::Logger()->info("Reading PPM");
    const fs::path ppmPath = parsed["ppm"].as<std::string>();
    auto ppm = vc::PerPixelMap::ReadPPM(ppmPath);

    // Setup output images
    const cv::Size size{
        static_cast<int>(ppm.width()), static_cast<int>(ppm.height())};
    cv::Mat pos = cv::Mat::zeros(size, CV_32FC3);
    cv::Mat norm = cv::Mat::zeros(size, CV_32FC3);

    // Fill the outputs
    vc::Logger()->info("Converting mappings...");
    for (const auto [y, x] : ppm.getMappingCoords()) {

        // Get values
        const auto m = ppm.getAsPixelMap(y, x);
        auto p = m.pos;
        auto n = m.normal;

        // Normalize position and surface normal
        // Surface normal is now +/- unit vector
        if (normalize) {
            cv::divide(p, dims, p);
            cv::normalize(n, n);
        }

        if (tspace) {
            // Get tangent and bitangent vectors
            cv::Vec3d tan{-1, -1, -1};
            if (ppm.hasMapping(m.y + 1, m.x)) {
                tan = ppm.getAsPixelMap(m.y + 1, m.x).pos - m.pos;
            } else if (ppm.hasMapping(m.y - 1, m.x)) {
                tan = m.pos - ppm.getAsPixelMap(m.y - 1, m.x).pos;
            }

            cv::Vec3d bitan{-1, -1, -1};
            if (ppm.hasMapping(m.y, m.x + 1)) {
                bitan = ppm.getAsPixelMap(m.y, m.x + 1).pos - m.pos;
            } else if (ppm.hasMapping(m.y, m.x - 1)) {
                bitan = m.pos - ppm.getAsPixelMap(m.y, m.x - 1).pos;
            }

            // Skip this pixel if we don't have the
            if (tan == cv::Vec3d{-1, -1, -1} ||
                bitan == cv::Vec3d{-1, -1, -1}) {
                vc::Logger()->warn(
                    "Can't calculate tangent/bitangent for ({},{})", m.x, m.y);
                n = cv::Vec3d::all(0);
            } else {
                cv::Mat w = cv::Mat::eye(3, 3, CV_64FC1);
                w.at<cv::Vec3d>(0, 0) = cv::normalize(tan);
                w.at<cv::Vec3d>(1, 0) = cv::normalize(bitan);
                w.at<cv::Vec3d>(2, 0) = cv::normalize(n);
                std::vector<cv::Vec3d> t{n};
                cv::transform(t, t, w);
                n = cv::normalize(t[0]);
            }
        }

        // Assign to output images
        pos.at<cv::Vec3f>(m.y, m.x) = cv::Vec3f{p};
        norm.at<cv::Vec3f>(m.y, m.x) = cv::Vec3f{n};
    }

    // Scale and shift the normal map to [0, 1]
    if (normalize) {
        vc::Logger()->info("Scaling and shifting normal map");
        norm *= 0.5F;
        norm += cv::Scalar(0.5F, 0.5F, 0.5F);
    }

    // WriteTIFF assumes 3-channel images are ZYX (BRG), but we've given XYZ
    // (RGB) Reverse the images channels to account for this
    cv::cvtColor(pos, pos, cv::COLOR_BGR2RGB);
    cv::cvtColor(norm, norm, cv::COLOR_BGR2RGB);

    // Write the images
    vc::Logger()->info("Saving images");
    auto prefix = parsed["output-prefix"].as<std::string>();
    vc::tiffio::WriteTIFF(prefix + "pos.tif", pos);
    vc::tiffio::WriteTIFF(prefix + "normal.tif", norm);
    vc::Logger()->info("Done.");
}