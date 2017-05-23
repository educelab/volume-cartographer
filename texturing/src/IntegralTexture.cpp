//
// Created by Seth on 11/24/16.
//
#include "vc/texturing/IntegralTexture.hpp"

#include <opencv2/core.hpp>

constexpr static uint16_t MAX_INTENSITY_16BPC =
    std::numeric_limits<uint16_t>::max();

using namespace volcart;
using namespace volcart::texturing;

Texture IntegralTexture::compute()
{
    // Setup
    result_ = Texture();

    auto height = static_cast<int>(ppm_.height());
    auto width = static_cast<int>(ppm_.width());

    // Setup the weights
    auto weights = setup_weights_();

    cv::Mat image = cv::Mat::zeros(height, width, CV_64FC1);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            // Skip this pixel if we have no mapping
            if (!ppm_.hasMapping(y, x)) {
                continue;
            }

            // Find the xyz coordinate of the original point
            auto pixelInfo = ppm_(y, x);
            cv::Vec3d xyz{pixelInfo[0], pixelInfo[1], pixelInfo[2]};
            cv::Vec3d xyzNorm{pixelInfo[3], pixelInfo[4], pixelInfo[5]};

            // Generate the neighborhood
            auto neighborhood = vol_->getVoxelNeighborsLinearInterpolated(
                xyz, xyzNorm, radius_, interval_, direction_);

            // Weight the neighborhood
            std::vector<double> weighted;
            cv::multiply(neighborhood, weights, weighted);

            // Sum the neighborhood
            auto value = std::accumulate(weighted.begin(), weighted.end(), 0.0);

            // Assign the intensity value at the UV position
            image.at<double>(y, x) = value;
        }
    }

    // Scale to uint16_t
    double min, max;
    cv::minMaxLoc(image, &min, &max);
    image.convertTo(
        image, CV_16U, MAX_INTENSITY_16BPC / (max - min),
        -min * MAX_INTENSITY_16BPC / (max - min));

    // Set output
    result_.addImage(image);
    result_.setPPM(ppm_);

    return result_;
}

std::vector<double> IntegralTexture::setup_weights_()
{
    // Neighborhood size
    size_t size = neighborhood_count_();

    // Linear Weighted Sum Setup
    double weight;
    double weightStep;
    switch (weightType_) {
        case Weight::None:
            weight = 1.0;
            weightStep = 0.0;
            break;
        // Favor the voxels along the negative normal
        case Weight::Negative:
            weight = 1.0;
            weightStep = -1.0 / size;
            break;
        // Favor the voxels along the positive normal
        case Weight::Positive:
            weight = 0.0;
            weightStep = 1.0 / size;
            break;
    }

    // Build a weight matrix
    std::vector<double> weights;
    weights.reserve(size);
    for (size_t i = 0; i < size; i++, weight += weightStep) {
        weights.push_back(weight);
    }

    return weights;
}
