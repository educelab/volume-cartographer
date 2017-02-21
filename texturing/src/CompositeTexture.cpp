//
// Created by Seth Parker on 10/20/15.
//

#include "vc/texturing/CompositeTexture.hpp"

namespace volcart
{
namespace texturing
{

CompositeTexture::CompositeTexture(
    ITKMesh::Pointer inputMesh,
    VolumePkg& volpkg,
    int outputWidth,
    int outputHeight,
    double radius,
    CompositeOption compositeMethod,
    DirectionOption compositeDirection)
    : input_(inputMesh)
    , volpkg_(volpkg)
    , width_(outputWidth)
    , height_(outputHeight)
    , radius_(radius)
    , method_(compositeMethod)
    , direction_(compositeDirection)
{
    ///// Generate UV Map /////
    // To-Do: Generate this map independent of point ordering - SP, 10/2015
    uvMap_ = volcart::texturing::SimpleUV(inputMesh, width_, height_);
    process_();
}

CompositeTexture::CompositeTexture(
    ITKMesh::Pointer inputMesh,
    VolumePkg& volpkg,
    UVMap uvMap,
    double radius,
    CompositeOption method,
    DirectionOption direction)
    : input_(inputMesh)
    , volpkg_(volpkg)
    , radius_(radius)
    , method_(method)
    , direction_(direction)
    , uvMap_(uvMap)
{
    width_ = static_cast<int>(uvMap.ratio().width);
    height_ = static_cast<int>(uvMap.ratio().height);
    process_();
}

int CompositeTexture::process_()
{
    ///// Generate Texture Image /////
    cv::Mat textureImage = cv::Mat::zeros(height_, width_, CV_16UC1);

    // Auto-generate minor radius for elliptical search
    double searchMinorRadius = radius_ / 3.0;
    if (searchMinorRadius < 1) {
        searchMinorRadius = 1;
    }

    // Iterate over all of the cells to lay out the faces in the output texture
    auto cells = input_->GetCells();
    for (auto cellIt = cells->Begin(); cellIt != cells->End(); ++cellIt) {
        // Link the pointer to our current cell
        auto cell = cellIt.Value();

        std::cerr << "volcart::CompositeTexture::message: Texturing face "
                  << cellIt.Index() << "/" << cells->End().Index() << "\r"
                  << std::flush;

        // Iterate over the vertices of the current cell
        for (auto pointsIt = cell->PointIdsBegin();
             pointsIt != cell->PointIdsEnd(); ++pointsIt) {
            auto pointID = *pointsIt;

            ITKPoint p = input_->GetPoint(pointID);
            ITKPixel normal;
            input_->GetPointData(pointID, &normal);

            // Fill in the output pixel with a value
            // cv::Mat.at uses (row, column)
            cv::Vec3d v1{p[0], p[1], p[2]};
            cv::Vec3d v2{normal[0], normal[1], normal[2]};
            double value = TextureWithMethod(
                v1, v2, volpkg_, method_, radius_, searchMinorRadius, 0.5,
                direction_);

            // Retrieve the point's uv position from the UV Map
            auto u = cvRound(uvMap_.get(pointID)[0] * (width_ - 1));
            auto v = cvRound(uvMap_.get(pointID)[1] * (height_ - 1));

            // Assign the intensity value at the UV position
            textureImage.at<uint16_t>(v, u) = static_cast<uint16_t>(value);
        }
    }
    std::cout << std::endl;

    // Assign and return the output
    texture_.addImage(textureImage);
    texture_.ppm().setUVMap(uvMap_);

    return EXIT_SUCCESS;
}
}  // texturing
}  // volcart
