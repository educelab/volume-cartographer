#include "vc/texturing/IntegralTexture.hpp"

#include <algorithm>
#include <cstddef>
#include <map>
#include <set>

#include <opencv2/core.hpp>

#include "vc/core/util/Iteration.hpp"

using namespace volcart;
using namespace volcart::texturing;

using Texture = IntegralTexture::Texture;

auto IntegralTexture::compute() -> Texture
{
    // Setup
    result_.clear();

    auto height = static_cast<int>(ppm_->height());
    auto width = static_cast<int>(ppm_->width());

    // Set up the weights
    setup_weights_();

    // Output image
    cv::Mat image = cv::Mat::zeros(height, width, CV_32FC1);

    // Get the mappings
    auto mappings = ppm_->getMappingCoords();

    // Sort the mappings by Z-value
    std::sort(
        mappings.begin(), mappings.end(),
        [&](const auto& lhs, const auto& rhs) {
            return (*ppm_)(lhs.y, lhs.x)[2] < (*ppm_)(rhs.y, rhs.x)[2];
        });

    // Iterate through the mappings
    progressStarted();
    for (const auto [idx, coord] : enumerate(mappings)) {
        progressUpdated(idx);

        // Generate the neighborhood
        const auto [y, x] = coord;
        const auto& m = ppm_->getMapping(y, x);
        const cv::Vec3d pos{m[0], m[1], m[2]};
        const cv::Vec3d normal{m[3], m[4], m[5]};
        auto n = gen_->compute(vol_, pos, {normal});

        // Clamp values
        if (clampToMax_) {
            std::replace_if(
                n.begin(), n.end(),
                [this](std::uint16_t v) { return v > clampMax_; }, clampMax_);
        }

        // Convert to double and weight the neighborhood
        NDArray<double> neighborhoodD(
            n.dims(), n.extents(), n.begin(), n.end());
        auto weighted = apply_weights_(neighborhoodD);

        // Sum the neighborhood
        auto value = std::accumulate(weighted.begin(), weighted.end(), 0.0);

        // Assign the intensity value at the UV position
        const auto v = static_cast<int>(y);
        const auto u = static_cast<int>(x);
        image.at<float>(v, u) = static_cast<float>(value);
    }
    progressComplete();

    cv::normalize(image, image, 0.0, 1.0, cv::NORM_MINMAX);

    // Set output
    result_.push_back(image);

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

auto IntegralTexture::apply_weights_(NDArray<double>& n) -> NDArray<double>
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
    auto extents = gen_->extents();
    linearWeights_ = NDArray<double>(gen_->dim(), extents);

    // Linear Weighted Sum Setup
    double weight;
    double weightStep;
    switch (linearWeight_) {
        // Favor the voxels along the negative normal
        case LinearWeightDirection::Negative:
            weight = 1.0;
            weightStep = -1.0 / extents[0];
            break;
        // Favor the voxels along the positive normal
        case LinearWeightDirection::Positive:
            weight = 0.0;
            weightStep = 1.0 / extents[0];
            break;
    }

    // Build a weight matrix
    for (auto& v : linearWeights_) {
        v = weight;
        weight += weightStep;
    }
}

auto IntegralTexture::apply_linear_weights_(NDArray<double>& n)
    -> NDArray<double>
{
    std::vector<double> weighted;
    cv::multiply(n.as_vector(), linearWeights_.as_vector(), weighted);
    return NDArray<double>(
        n.dims(), n.extents(), weighted.begin(), weighted.end());
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

auto IntegralTexture::expodiff_intersection_pts_() -> std::vector<std::uint16_t>
{
    // Get all the intensity values
    std::vector<std::uint16_t> values;
    for (const auto [y, x] : ppm_->getMappingCoords()) {
        const auto& m = ppm_->getMapping(y, x);
        values.emplace_back(vol_->interpolateAt({m[0], m[1], m[2]}));
    }

    return values;
}

auto IntegralTexture::expodiff_mean_base_() -> double
{
    // Get intersection points
    auto values = expodiff_intersection_pts_();

    // Calculate the mean
    std::size_t n = 0;
    double mean = 0.0;
    for (const auto& v : values) {
        double delta = v - mean;
        mean += delta / static_cast<double>(++n);
    }

    return mean;
}

auto IntegralTexture::expodiff_mode_base_() -> double
{
    // Get intersection points
    auto values = expodiff_intersection_pts_();

    // Generate a histogram
    std::map<std::uint16_t, int> histogram;
    for (const auto& v : values) {
        try {
            histogram.at(v) += 1;
        } catch (const std::out_of_range&) {
            histogram[v] = 1;
        }
    }

    // Sort by frequency high to low
    using KVPair = std::pair<std::uint16_t, int>;
    using Comparator = std::function<bool(KVPair, KVPair)>;
    Comparator compare = [](KVPair a, KVPair b) { return a.second > b.second; };
    std::set<KVPair, Comparator> sorter(
        histogram.begin(), histogram.end(), compare);

    // Return the top candidate
    return sorter.begin()->first;
}

auto IntegralTexture::apply_expodiff_weights_(NDArray<double>& n) const
    -> NDArray<double>
{
    std::vector<double> vals;
    for (const auto& val : n) {
        if (suppressBelowBase_ && expoDiffBase_ >= val) {
            vals.emplace_back(0);
            continue;
        }
        double diff = std::abs(val - expoDiffBase_);
        vals.emplace_back(std::pow(diff, expoDiffExponent_));
    }

    return NDArray<double>(n.dims(), n.extents(), vals.begin(), vals.end());
}

auto IntegralTexture::New() -> IntegralTexture::Pointer
{
    return std::make_shared<IntegralTexture>();
}

void IntegralTexture::setGenerator(NeighborhoodGenerator::Pointer g)
{
    gen_ = std::move(g);
}

void IntegralTexture::setClampValuesToMax(bool b) { clampToMax_ = b; }

auto IntegralTexture::clampValuesToMax() const -> bool { return clampToMax_; }

void IntegralTexture::setClampMax(std::uint16_t m) { clampMax_ = m; }

auto IntegralTexture::clampMax() const -> std::uint16_t { return clampMax_; }

void IntegralTexture::setWeightMethod(IntegralTexture::WeightMethod w)
{
    weight_ = w;
}

auto IntegralTexture::weightMethod() const -> IntegralTexture::WeightMethod
{
    return weight_;
}

void IntegralTexture::setLinearWeightDirection(
    IntegralTexture::LinearWeightDirection w)
{
    linearWeight_ = w;
}

auto IntegralTexture::linearWeightDirection() const
    -> IntegralTexture::LinearWeightDirection
{
    return linearWeight_;
}

void IntegralTexture::setExponentialDiffExponent(int e)
{
    expoDiffExponent_ = e;
}

auto IntegralTexture::exponentialDiffExponent() const -> int
{
    return expoDiffExponent_;
}

void IntegralTexture::setExponentialDiffBaseMethod(
    IntegralTexture::ExpoDiffBaseMethod m)
{
    expoDiffBaseMethod_ = m;
}

auto IntegralTexture::exponentialDiffBaseMethod() const
    -> IntegralTexture::ExpoDiffBaseMethod
{
    return expoDiffBaseMethod_;
}

void IntegralTexture::setExponentialDiffBaseValue(double b)
{
    expoDiffManualBase_ = b;
}

auto IntegralTexture::exponentialDiffBaseValue() const -> double
{
    return expoDiffManualBase_;
}

void IntegralTexture::setExponentialDiffSuppressBelowBase(bool b)
{
    suppressBelowBase_ = b;
}

auto IntegralTexture::exponentialDiffSuppressBelowBase() const -> bool
{
    return suppressBelowBase_;
}
