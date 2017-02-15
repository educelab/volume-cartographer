#include <iostream>

#include "vc/segmentation/lrps/Derivative.hpp"
#include "vc/segmentation/lrps/EnergyMetrics.hpp"

using namespace volcart::segmentation;

// Calculates the active contour internal energy. See:
// https://en.wikipedia.org/wiki/Active_contour_model#Internal_energy
// Note: k1 and k2 are constant for all particles in the curve
double EnergyMetrics::ActiveContourInternal(
    const FittedCurve& curve, double k1, double k2)
{
    if (curve.size() <= 0) {
        return 0;
    }

    auto d1Current = NormalizeVector(D1(curve.points()));
    auto d2Current = NormalizeVector(D2(curve.points()));

    double intE = 0;
    for (auto p : Zip(d1Current, d2Current)) {
        intE += k1 * std::pow(cv::norm(p.first), 2) +
                k2 * std::pow(cv::norm(p.second), 2);
    }

    return intE / (2 * curve.size());
}

// Amalgamation of energy metrics used parameterized by their coefficients.
// To disable a metric, simply set its coefficient to zero
double EnergyMetrics::TotalEnergy(
    const FittedCurve& curve,
    double alpha,
    double k1,
    double k2,
    double beta,
    double delta)
{
    auto intE = EnergyMetrics::ActiveContourInternal(curve, k1, k2);
    auto kE = EnergyMetrics::AbsCurvatureSum(curve);
    auto sE = EnergyMetrics::WindowedArcLength(curve, 3);
    return alpha * intE + beta * kE + delta * sE;
}

// Sum of the absolute value of the curvature from curve
double EnergyMetrics::AbsCurvatureSum(const FittedCurve& curve)
{
    if (curve.size() <= 0) {
        return 0;
    }

    auto k = curve.curvature();
    std::transform(std::begin(k), std::end(k), std::begin(k), [](auto e) {
        return std::abs(e);
    });
    k = NormalizeVector(k);
    return std::accumulate(
               begin(k), end(k), 0.0,
               [](double sum, double d) { return sum + d; }) /
           curve.size();
}

// Determine arc length across a window of size 'windowSize' centered at
// 'index'
double EnergyMetrics::LocalWindowedArcLength(
    const FittedCurve& curve, int index, int windowSize)
{
    if (curve.size() <= 0) {
        return 0;
    }

    if (index < 0 || index >= int(curve.size())) {
        auto msg = "index '" + std::to_string(index) + "' outside curve range";
        throw std::invalid_argument(msg);
    } else if (windowSize < 0 || windowSize >= int(curve.size())) {
        auto msg = "invalid windowSize";
        throw std::invalid_argument(msg);
    }

    int windowRadius = windowSize / 2;
    double sum = 0;
    int lastIdx = curve.size() - 1;
    for (int i = index - windowRadius; i < index + windowRadius; ++i) {
        if (i < 0) {
            sum += cv::norm(curve(-i), curve(-(i + 1)));
        } else if (i >= int(curve.size()) || i + 1 >= int(curve.size())) {
            int iDiff = i - lastIdx;
            sum +=
                cv::norm(curve(lastIdx - iDiff), curve(lastIdx - (iDiff + 1)));
        } else {
            sum += cv::norm(curve(i), curve(i + 1));
        }
    }

    // Average distance between 2 points on curve
    double avgDist = curve.arclength() / (curve.size() - 1);
    return sum / avgDist;
}

// Apply LocalWindowedArcLength across the entire curve
double EnergyMetrics::WindowedArcLength(
    const FittedCurve& curve, int windowSize)
{
    // Special case - empty curve
    if (curve.size() <= 0) {
        return 0;
    }

    double sum = 0;
    for (size_t i = 0; i < curve.size(); ++i) {
        sum += EnergyMetrics::LocalWindowedArcLength(curve, i, windowSize);
    }
    return sum / curve.size();
}
