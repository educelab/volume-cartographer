#include "vc/texturing/CompositeTexture.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>

#include "vc/core/util/FloatComparison.hpp"
#include "vc/core/util/Iteration.hpp"

using namespace volcart;
using namespace volcart::texturing;

using Texture = CompositeTexture::Texture;
using Filter = CompositeTexture::Filter;

namespace
{
constexpr double MEDIAN_MEAN_PERCENT_RANGE{0.70};

auto FilterMin(Neighborhood n) -> std::uint16_t
{
    return *std::min_element(n.begin(), n.end());
}

auto FilterMax(Neighborhood n) -> std::uint16_t
{
    return *std::max_element(n.begin(), n.end());
}

auto FilterMedian(Neighborhood n) -> std::uint16_t
{
    std::nth_element(n.begin(), n.begin() + n.size() / 2, n.end());
    return n(n.size() / 2);
}

auto FilterMean(Neighborhood n) -> std::uint16_t
{
    auto sum = std::accumulate(std::begin(n), std::end(n), double{0});
    return static_cast<std::uint16_t>(std::round(sum / n.size()));
}

auto FilterMedianMean(Neighborhood n, double range) -> std::uint16_t
{
    // If the range is 1.0, it's just a normal mean operation
    if (AlmostEqual<double>(range, 1.0)) {
        return FilterMean(n);
    }
    if (AlmostEqual<double>(range, 0.0)) {
        return 0;
    }

    // Sort
    std::sort(n.begin(), n.end());

    // The number of things we're going to sum
    auto count = static_cast<std::size_t>(std::ceil(n.size() * range));
    // The number of things before we start summing
    auto offset =
        static_cast<std::size_t>(std::floor((n.size() - count) / 2.0));

    // Sum
    auto sum = std::accumulate(
        n.begin() + offset, n.begin() + offset + count, double{0});

    // Average
    return static_cast<std::uint16_t>(std::round(sum / count));
}

auto ApplyFilter(const Neighborhood& n, Filter filter) -> std::uint16_t
{
    switch (filter) {
        case Filter::Minimum:
            return FilterMin(n);
        case Filter::Maximum:
            return FilterMax(n);
        case Filter::Median:
            return FilterMedian(n);
        case Filter::Mean:
            return FilterMean(n);
        case Filter::MedianAverage:
            return FilterMedianMean(n, MEDIAN_MEAN_PERCENT_RANGE);
    }
}

}  // namespace

auto CompositeTexture::New() -> CompositeTexture::Pointer
{
    return std::make_shared<CompositeTexture>();
}

void CompositeTexture::setGenerator(NeighborhoodGenerator::Pointer g)
{
    gen_ = std::move(g);
}

void CompositeTexture::setFilter(CompositeTexture::Filter f) { filter_ = f; }

auto CompositeTexture::compute() -> Texture
{
    if (gen_->dim() < 1) {
        throw std::runtime_error("Generator dimension below required");
    }

    // Setup
    result_.clear();
    auto height = static_cast<int>(ppm_->height());
    auto width = static_cast<int>(ppm_->width());

    // Output image
    cv::Mat image = cv::Mat::zeros(height, width, CV_16UC1);

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
        auto neighborhood = gen_->compute(vol_, pos, {normal});
        Neighborhood::Flatten(neighborhood, 1);

        // Assign the intensity value at the UV position
        const auto v = static_cast<int>(y);
        const auto u = static_cast<int>(x);
        image.at<std::uint16_t>(v, u) = ::ApplyFilter(neighborhood, filter_);
    }
    progressComplete();

    // Set output
    result_.push_back(image);

    return result_;
}
