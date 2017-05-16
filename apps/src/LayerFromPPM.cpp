//
// Created by Seth Parker on 3/23/17.
//

#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>

#include "vc/core/types/PerPixelMap.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/texturing/LayerTexture.hpp"

namespace vc = volcart;
namespace fs = boost::filesystem;

int main(int argc, char* argv[])
{
    if (argc < 6) {
        std::cout << "Usage: " << argv[0];
        std::cout << " [volpkg] [input.ppm] [radius] [interval] [output-dir]"
                  << std::endl;
        return EXIT_FAILURE;
    }

    // Get inputs
    fs::path vpkgPath = fs::canonical(argv[1]);
    fs::path ppmPath = argv[2];
    auto radius = std::stoi(argv[3]);
    auto interval = std::stod(argv[4]);
    fs::path outputDir = argv[5];

    // Load volpkg
    vc::VolumePkg vpkg(vpkgPath);

    // Load ppm
    std::cout << "Loading per-pixel map..." << std::endl;
    auto ppm = vc::PerPixelMap::ReadPPM(ppmPath);

    // Layer texture
    std::cout << "Generating layers..." << std::endl;
    vc::texturing::LayerTexture s;
    s.setVolume(vpkg.volume());
    s.setPerPixelMap(ppm);
    s.setSamplingRadius(radius);
    s.setSamplingInterval(interval);
    auto texture = s.compute();

    std::cout << "Writing layers..." << std::endl;
    fs::path filepath;
    for (size_t i = 0; i < texture.numberOfImages(); ++i) {
        filepath = outputDir / (std::to_string(i) + ".png");
        cv::imwrite(filepath.string(), texture.image(i));
    }
}