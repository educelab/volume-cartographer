#include "vc/texturing/CompositeTexture.hpp"

#include <algorithm>

#include "vc/core/util/FloatComparison.hpp"

using namespace volcart;
using namespace volcart::texturing;
using namespace std::chrono_literals;

using Texture = CompositeTexture::Texture;
using Filter = CompositeTexture::Filter;

namespace
{
constexpr double MEDIAN_MEAN_PERCENT_RANGE{0.70};

auto FilterMin(Neighborhood n) -> uint16_t
{
    return *std::min_element(n.begin(), n.end());
}

auto FilterMax(Neighborhood n) -> uint16_t
{
    return *std::max_element(n.begin(), n.end());
}

auto FilterMedian(Neighborhood n) -> uint16_t
{
    std::nth_element(n.begin(), n.begin() + n.size() / 2, n.end());
    return n(n.size() / 2);
}

auto FilterMean(Neighborhood n) -> uint16_t
{
    auto sum = std::accumulate(std::begin(n), std::end(n), double{0});
    return static_cast<uint16_t>(std::round(sum / n.size()));
}

auto FilterMedianMean(Neighborhood n, double range) -> uint16_t
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
    auto count = static_cast<size_t>(std::ceil(n.size() * range));
    // The number of things before we start summing
    auto offset = static_cast<size_t>(std::floor((n.size() - count) / 2.0));

    // Sum
    auto sum = std::accumulate(
        n.begin() + offset, n.begin() + offset + count, double{0});

    // Average
    return static_cast<uint16_t>(std::round(sum / count));
}

auto ApplyFilter(const Neighborhood& n, Filter filter) -> uint16_t
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

void DoTexture(
    cv::Mat& image,
    const PerPixelMap::PixelMap& pixel,
    const Filter& filter,
    const NeighborhoodGenerator::Pointer& generator,
    Volume::Pointer& volume)
{
    // Generate the neighborhood
    auto neighborhood = generator->compute(volume, pixel.pos, {pixel.normal});
    Neighborhood::Flatten(neighborhood, 1);

    // Assign the intensity value at the UV position
    auto y = static_cast<int>(pixel.y);
    auto x = static_cast<int>(pixel.x);
    image.at<uint16_t>(y, x) = ::ApplyFilter(neighborhood, filter);
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
    auto mappings = ppm_->getMappings();

    // Sort the mappings by Z-value
    std::sort(
        mappings.begin(), mappings.end(), [](const auto& lhs, const auto& rhs) {
            return lhs.pos[2] < rhs.pos[2];
        });

    // Iterate through the mappings
    size_t counter = 0;
    progressStarted();
    for (const auto& pixel : mappings) {
        progressUpdated(counter++);

        // Generate the neighborhood
        auto neighborhood = gen_->compute(vol_, pixel.pos, {pixel.normal});
        Neighborhood::Flatten(neighborhood, 1);

        // Assign the intensity value at the UV position
        auto y = static_cast<int>(pixel.y);
        auto x = static_cast<int>(pixel.x);
        image.at<uint16_t>(y, x) = ::ApplyFilter(neighborhood, filter_);
    }
    progressComplete();

    // Set output
    result_.push_back(image);

    return result_;
}
