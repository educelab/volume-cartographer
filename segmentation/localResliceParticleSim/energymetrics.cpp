#include "energymetrics.h"
#include "derivative.h"

using namespace volcart::segmentation;

// Calculates the active contour internal energy. See:
// https://en.wikipedia.org/wiki/Active_contour_model#Internal_energy
// Note: k1 and k2 are constant for all particles in the curve
double EnergyMetrics::ActiveContourInternal(const FittedCurve& curve,
                                            double k1,
                                            double k2)
{
    auto d1current = normalizeVector(d1(curve.points()));
    auto d2current = normalizeVector(d2(curve.points()));

    double intE = 0;
    for (auto p : zip(d1current, d2current)) {
        Voxel d1sq, d2sq;
        cv::pow(p.first, 2, d1sq);
        cv::pow(p.second, 2, d2sq);
        intE += k1 * cv::norm(d1sq) + k2 * cv::norm(d2sq);
    }

    return intE / (2 * curve.size());
}

// Amalgamation of energy metrics used parameterized by their coefficients.
// To disable a metric, simply set its coefficient to zero
double EnergyMetrics::TotalEnergy(const FittedCurve& curve,
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
    auto k = normalizeVector(curve.curvature());
    return std::accumulate(begin(k), end(k), 0.0, [](double sum, double d) {
               return sum + std::abs(d);
           }) / curve.size();
}

// Determine arc length across a window of size 'windowSize' centered at
// 'index'
double EnergyMetrics::LocalWindowedArcLength(const FittedCurve& curve,
                                             int32_t index,
                                             int32_t windowSize)
{
    int32_t windowRadius = windowSize / 2;
    double sum = 0;
    int32_t lastIdx = curve.size() - 1;
    for (int32_t i = index - windowRadius; i < index + windowRadius; ++i) {
        if (i < 0) {
            sum += cv::norm(curve(-i), curve(-(i + 1)));
        } else if (i >= int32_t(curve.size()) ||
                   i + 1 >= int32_t(curve.size())) {
            int32_t iDiff = i - lastIdx;
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
double EnergyMetrics::WindowedArcLength(const FittedCurve& curve,
                                        int32_t windowSize)
{
    double sum = 0;
    for (size_t i = 0; i < curve.size(); ++i) {
        sum += EnergyMetrics::LocalWindowedArcLength(curve, i, windowSize);
    }
    return sum / curve.size();
}

/*
// Sums up distances of each particle from its two nearest neighbors. At the
// ends of the curve, simply reflect distances to their singular neighbors
 double EnergyMetrics::TwoNearestNeighborDistanceSum(
    const FittedCurve& curve)
{
    auto vs = curve.points();
    std::vector<double> diffs(vs.size());

    // Special case for first and last elements. Since they don't have
    // neighbors, just double their distances to the elements next to them. This
    // might not be the best, but it's what works for now
    diffs.front() = 2 * cv::norm(vs[0], vs[1]);
    diffs.back() = 2 * cv::norm(vs.back(), vs.rbegin()[1]);

    // Handle the rest of the vector
    for (uint32_t i = 1; i < vs.size() - 1; ++i) {
        diffs[i] = cv::norm(vs[i - 1], vs[i]) + cv::norm(vs[i], vs[i + 1]);
    }

    // Normalize and sum
    diffs = normalizeVector(diffs);
    return std::accumulate(begin(diff), end(diff), 0.0) / (curve.size() - 1);
}
*/
