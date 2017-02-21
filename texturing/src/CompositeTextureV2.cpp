//
// Created by Seth Parker on 12/28/15.
//

#include "vc/texturing/CompositeTextureV2.hpp"
#include "vc/texturing/PPMGenerator.hpp"

namespace volcart
{
namespace texturing
{

// Constructor
CompositeTextureV2::CompositeTextureV2(
    ITKMesh::Pointer inputMesh,
    VolumePkg& volpkg,
    UVMap uvMap,
    double radius,
    size_t width,
    size_t height,
    CompositeOption method,
    DirectionOption direction)
    : input_{inputMesh}
    , volpkg_{volpkg}
    , width_{width}
    , height_{height}
    , radius_{radius}
    , method_{method}
    , direction_{direction}
    , uvMap_{std::move(uvMap)}
{
    process_();
}

// Do the hard work
int CompositeTextureV2::process_()
{
    // Auto-generate minor radius for elliptical search
    double searchMinorRadius = radius_ / 3;
    if (searchMinorRadius < 1) {
        searchMinorRadius = 1;
    }

    // Generate PPM
    PPMGenerator gen(height_, width_);
    gen.setMesh(input_);
    gen.setUVMap(uvMap_);
    gen.compute();
    auto ppm = gen.getPPM();

    // Iterate over every pixel in the output image
    cv::Mat image = cv::Mat::zeros(height_, width_, CV_16UC1);
    for (size_t y = 0; y < height_; ++y) {
        for (size_t x = 0; x < width_; ++x) {
            auto progress =
                (x + 1.0 + (width_ * y)) * 100.0 / (width_ * height_);
            std::cerr << "volcart::texturing::compositeTexturing: Generating "
                         "texture: "
                      << std::to_string(progress) << "%\r" << std::flush;

            // Skip this pixel if we have no mapping
            if (!ppm.hasMapping(y, x)) {
                continue;
            }

            // Find the xyz coordinate of the original point
            auto pixelInfo = ppm(y, x);

            cv::Vec3d xyz{pixelInfo[0], pixelInfo[1], pixelInfo[2]};
            cv::Vec3d xyzNorm{pixelInfo[3], pixelInfo[4], pixelInfo[5]};

            // Generate the intensity value
            auto value = TextureWithMethod(
                xyz, xyzNorm, volpkg_, method_, radius_, searchMinorRadius, 0.5,
                direction_);

            // Assign the intensity value at the UV position
            image.at<uint16_t>(y, x) = static_cast<uint16_t>(value);
        }
    }
    std::cerr << std::endl;

    // Set output
    texture_.addImage(image);
    texture_.setPPM(ppm);

    return EXIT_SUCCESS;
};
}
}
