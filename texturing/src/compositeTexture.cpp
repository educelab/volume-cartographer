//
// Created by Seth Parker on 10/20/15.
//

#include "texturing/compositeTexture.h"

namespace volcart
{
namespace texturing
{
compositeTexture::compositeTexture(
    ITKMesh::Pointer inputMesh,
    VolumePkg& volpkg,
    int output_w,
    int output_h,
    double radius,
    CompositeOption compositeMethod,
    DirectionOption compositeDirection)
    : _input(inputMesh)
    , _volpkg(volpkg)
    , _width(output_w)
    , _height(output_h)
    , _radius(radius)
    , _method(compositeMethod)
    , _direction(compositeDirection)
{
    ///// Generate UV Map /////
    // To-Do: Generate this map independent of point ordering - SP, 10/2015
    _uvMap = volcart::texturing::simpleUV(inputMesh, _width, _height);
    _process();
}

compositeTexture::compositeTexture(
    ITKMesh::Pointer inputMesh,
    VolumePkg& volpkg,
    UVMap uvMap,
    double radius,
    CompositeOption method,
    DirectionOption direction)
    : _input(inputMesh)
    , _volpkg(volpkg)
    , _radius(radius)
    , _method(method)
    , _direction(direction)
    , _uvMap(uvMap)
{
    _width = static_cast<int>(uvMap.ratio().width);
    _height = static_cast<int>(uvMap.ratio().height);

    _process();
}

int compositeTexture::_process()
{

    ///// Generate Texture Image /////
    cv::Mat textureImage = cv::Mat::zeros(_height, _width, CV_16UC1);

    // Auto-generate minor radius for elliptical search
    double searchMinorRadius;
    if ((searchMinorRadius = _radius / 3) < 1)
        searchMinorRadius = 1;

    // Initialize iterators
    ITKCellIterator cellIterator = _input->GetCells()->Begin();
    ITKCellIterator cellEnd = _input->GetCells()->End();
    ITKCell* cell;
    ITKPointInCellIterator pointsIterator;

    unsigned long pointID;
    int u, v;

    // Iterate over all of the cells to lay out the faces in the output texture
    for (; cellIterator != cellEnd; ++cellIterator) {
        // Link the pointer to our current cell
        cell = cellIterator.Value();

        std::cerr << "volcart::compositeTexture::message: Texturing face "
                  << cellIterator.Index() << "/" << cellEnd.Index() << "\r"
                  << std::flush;

        // Iterate over the vertices of the current cell
        pointsIterator = cell->PointIdsBegin();
        for (; pointsIterator != cell->PointIdsEnd(); ++pointsIterator) {
            pointID = *pointsIterator;

            ITKPoint p = _input->GetPoint(pointID);
            ITKPixel normal;
            _input->GetPointData(pointID, &normal);

            // Fill in the output pixel with a value
            // cv::Mat.at uses (row, column)
            double value = textureWithMethod(
                cv::Vec3d(p[0], p[1], p[2]),
                cv::Vec3d(normal[0], normal[1], normal[2]), _volpkg, _method,
                _radius, searchMinorRadius, 0.5, _direction);

            // Retrieve the point's uv position from the UV Map
            u = cvRound(_uvMap.get(pointID)[0] * (_width - 1));
            v = cvRound(_uvMap.get(pointID)[1] * (_height - 1));

            // Assign the intensity value at the UV position
            textureImage.at<uint16_t>(v, u) = static_cast<uint16_t>(value);
        }
    }
    std::cout << std::endl;

    // Assign and return the output
    _texture.addImage(textureImage);
    _texture.ppm().setUVMap(_uvMap);

    return EXIT_SUCCESS;
}
}  // texturing
}  // volcart
