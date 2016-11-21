//
// Created by Seth Parker on 12/28/15.
//

#include "texturing/compositeTextureV2.h"
#include "texturing/PPMGenerator.h"

namespace volcart
{
namespace texturing
{

// Constructor
compositeTextureV2::compositeTextureV2(
    ITKMesh::Pointer inputMesh,
    VolumePkg& volpkg,
    UVMap uvMap,
    double radius,
    int width,
    int height,
    CompositeOption method,
    DirectionOption direction)
    : _volpkg(volpkg)
    , _input(inputMesh)
    , _uvMap(uvMap)
    , _radius(radius)
    , _width(width)
    , _height(height)
    , _method(method)
    , _direction(direction)
{
    _process();
};

// Do the hard work
int compositeTextureV2::_process()
{

    // Auto-generate minor radius for elliptical search
    double searchMinorRadius;
    if ((searchMinorRadius = _radius / 3) < 1)
        searchMinorRadius = 1;

    // Generate PPM
    PPMGenerator gen(_height, _width);
    gen.setMesh(_input);
    gen.setUVMap(_uvMap);
    gen.compute();
    PerPixelMap ppm = gen.getPPM();

    // Iterate over every pixel in the output image
    cv::Mat image = cv::Mat::zeros(_height, _width, CV_16UC1);

    for (int y = 0; y < _height; ++y) {
        for (int x = 0; x < _width; ++x) {
            double progress = (double)(x + 1 + (_width * y)) /
                              (double)(_width * _height) * (double)100;
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
            // Use the cell normal as the normal for this point
            cv::Vec3d xyz_norm{pixelInfo[3], pixelInfo[4], pixelInfo[5]};

            // Generate the intensity value
            double value = textureWithMethod(
                xyz, xyz_norm, _volpkg, _method, _radius, searchMinorRadius,
                0.5, _direction);

            // Assign the intensity value at the UV position
            image.at<unsigned short>(y, x) = (unsigned short)value;
        }
    }
    std::cerr << std::endl;

    // Set output
    _texture.addImage(image);
    _texture.setMap(ppm);

    return EXIT_SUCCESS;
};
}
}
