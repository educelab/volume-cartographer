//
// Created by Seth Parker on 12/28/15.
//

#include "texturing/CompositeTextureV2.h"
#include "texturing/PPMGenerator.h"

namespace volcart
{
namespace texturing
{

// Constructor
compositeTextureV2::compositeTextureV2(
    ITKMesh::Pointer m,
    VolumePkg& v,
    UVMap uv,
    double r,
    size_t w,
    size_t h,
    CompositeOption o,
    DirectionOption d)
    : _input{m}
    , _volpkg{v}
    , _width{w}
    , _height{h}
    , _radius{r}
    , _method{o}
    , _direction{d}
    , _uvMap{uv}
{
    _process();
}

// Do the hard work
int compositeTextureV2::_process()
{
    // Auto-generate minor radius for elliptical search
    double searchMinorRadius = _radius / 3;
    if (searchMinorRadius < 1) {
        searchMinorRadius = 1;
    }

    // Generate PPM
    PPMGenerator gen(_height, _width);
    gen.setMesh(_input);
    gen.setUVMap(_uvMap);
    gen.compute();
    auto ppm = gen.getPPM();

    // Iterate over every pixel in the output image
    cv::Mat image = cv::Mat::zeros(_height, _width, CV_16UC1);
    for (size_t y = 0; y < _height; ++y) {
        for (size_t x = 0; x < _width; ++x) {
            auto progress =
                (x + 1.0 + (_width * y)) * 100.0 / (_width * _height);
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
            cv::Vec3d xyz_norm{pixelInfo[3], pixelInfo[4], pixelInfo[5]};

            // Generate the intensity value
            auto value = textureWithMethod(
                xyz, xyz_norm, _volpkg, _method, _radius, searchMinorRadius,
                0.5, _direction);

            // Assign the intensity value at the UV position
            image.at<uint16_t>(y, x) = static_cast<uint16_t>(value);
        }
    }
    std::cerr << std::endl;

    // Set output
    _texture.addImage(image);
    _texture.setPPM(ppm);

    return EXIT_SUCCESS;
};
}
}
