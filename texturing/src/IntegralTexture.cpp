//
// Created by Seth on 11/24/16.
//
#include "vc/texturing/IntegralTexture.hpp"

#include <algorithm>
#include <map>
#include <set>

#include <opencv2/core.hpp>

using namespace volcart;
using namespace volcart::texturing;

using DoublesVec = std::vector<double>;

Texture IntegralTexture::compute()
{
    // Setup
    result_ = Texture();

    auto height = static_cast<int>(ppm_.height());
    auto width = static_cast<int>(ppm_.width());

    // Setup the weights
    setup_weights_();

    // Output image
    cv::Mat image = cv::Mat::zeros(height, width, CV_64FC1);

    // Get the mappings
    auto mappings = ppm_.getMappings();

    // Sort the mappings by Z-value
    std::sort(
        mappings.begin(), mappings.end(), [](const auto& lhs, const auto& rhs) {
            return lhs.pos[2] < rhs.pos[2];
        });

    // Iterate through the mappings
    for (const auto& pixel : mappings) {
        // Generate the neighborhood
        auto neighborhood = vol_->getVoxelNeighborsLinearInterpolated(
            pixel.pos, pixel.normal, radius_, interval_, direction_);

        // Clamp values
        if (clampToMax_) {
            std::replace_if(
                neighborhood.begin(), neighborhood.end(),
                [this](uint16_t v) { return v > clampMax_; }, clampMax_);
        }

        // Convert to double and weight the neighborhood
        DoublesVec neighborhoodD(neighborhood.begin(), neighborhood.end());
        auto weighted = apply_weights_(neighborhoodD);

        // Sum the neighborhood
        auto value = std::accumulate(weighted.begin(), weighted.end(), 0.0);

        // Assign the intensity value at the UV position
        image.at<double>(pixel.y, pixel.x) = value;
    }

    cv::normalize(image, image, 0.0, 1.0, cv::NORM_MINMAX);

    // Set output
    result_.addImage(image);
    result_.setPPM(ppm_);

    return result_;
}

///// Setup and Apply weights generally /////
void IntegralTexture::setup_weights_()
{
    switch (weight_) {
        case WeightMethod::None:
            return;
        case WeightMethod::Linear:
            return setup_linear_weights_();
        case WeightMethod::ExpoDiff:
            return setup_expodiff_weights_();
    }
}

DoublesVec IntegralTexture::apply_weights_(DoublesVec& n)
{
    switch (weight_) {
        case WeightMethod::None:
            return n;
        case WeightMethod::Linear:
            return apply_linear_weights_(n);
        case WeightMethod::ExpoDiff:
            return apply_expodiff_weights_(n);
    }
}

///// Linear weighting /////
void IntegralTexture::setup_linear_weights_()
{
    // Neighborhood size
    size_t size = neighborhood_count_();

    // Linear Weighted Sum Setup
    double weight;
    double weightStep;
    switch (linearWeight_) {
        // Favor the voxels along the negative normal
        case LinearWeightDirection::Negative:
            weight = 1.0;
            weightStep = -1.0 / size;
            break;
        // Favor the voxels along the positive normal
        case LinearWeightDirection::Positive:
            weight = 0.0;
            weightStep = 1.0 / size;
            break;
    }

    // Build a weight matrix
    linearWeights_.clear();
    linearWeights_.reserve(size);
    for (size_t i = 0; i < size; i++) {
        linearWeights_.push_back(weight);
        weight += weightStep;
    }
}

DoublesVec IntegralTexture::apply_linear_weights_(DoublesVec& n)
{
    std::vector<double> weighted;
    cv::multiply(n, linearWeights_, weighted);
    return weighted;
}

///// Exponential Difference weighting /////
void IntegralTexture::setup_expodiff_weights_()
{
    switch (expoDiffBaseMethod_) {
        case ExpoDiffBaseMethod::Manual:
            expoDiffBase_ = expoDiffManualBase_;
            return;
        case ExpoDiffBaseMethod::Mean:
            expoDiffBase_ = expodiff_mean_base_();
            return;
        case ExpoDiffBaseMethod::Mode:
            expoDiffBase_ = expodiff_mode_base_();
            return;
    }
}

std::vector<uint16_t> IntegralTexture::expodiff_intersection_pts_()
{
    // Get all of the intensity values
    std::vector<uint16_t> values;
    auto height = ppm_.height();
    auto width = ppm_.width();
    for (size_t y = 0; y < height; ++y) {
        for (size_t x = 0; x < width; ++x) {
            // Skip this pixel if we have no mapping
            if (!ppm_.hasMapping(y, x)) {
                continue;
            }

            // Find the xyz coordinate of the original point
            auto pixelInfo = ppm_(y, x);
            cv::Vec3d xyz{pixelInfo[0], pixelInfo[1], pixelInfo[2]};

            values.emplace_back(vol_->interpolatedIntensityAt(xyz));
        }
    }

    return values;
}

double IntegralTexture::expodiff_mean_base_()
{
    // Get intersection points
    auto values = expodiff_intersection_pts_();

    // Calculate the mean
    size_t n = 0;
    double mean = 0.0;
    for (const auto& v : values) {
        double delta = v - mean;
        mean += delta / ++n;
    }

    return mean;
}

double IntegralTexture::expodiff_mode_base_()
{
    // Get intersection points
    auto values = expodiff_intersection_pts_();

    // Generate a histogram
    std::map<uint16_t, int> histogram;
    for (const auto& v : values) {
        try {
            histogram.at(v) += 1;
        } catch (const std::out_of_range&) {
            histogram[v] = 1;
        }
    }

    // Sort by frequency high to low
    using KVPair = std::pair<uint16_t, int>;
    using Comparator = std::function<bool(KVPair, KVPair)>;
    Comparator compare = [](KVPair a, KVPair b) { return a.second > b.second; };
    std::set<KVPair, Comparator> sorter(
        histogram.begin(), histogram.end(), compare);

    // Return the top candidate
    return sorter.begin()->first;
}

DoublesVec IntegralTexture::apply_expodiff_weights_(std::vector<double>& n)
{
    std::vector<double> weighted;
    for (const auto& val : n) {
        if (suppressBelowBase_ && expoDiffBase_ >= val) {
            continue;
        }
        double diff = std::abs(val - expoDiffBase_);
        weighted.emplace_back(std::pow(diff, expoDiffExponent_));
    }

    return weighted;
}
