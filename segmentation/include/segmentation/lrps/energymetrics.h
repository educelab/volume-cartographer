#pragma once

#ifndef VOLCART_SEGMENTATION_ENERGY_METRICS
#define VOLCART_SEGMENTATION_ENERGY_METRICS

#include "segmentation/lrps/fittedcurve.h"

namespace volcart
{
namespace segmentation
{
class EnergyMetrics
{
public:
    // Calculates the active contour internal energy. See:
    // https://en.wikipedia.org/wiki/Active_contour_model#Internal_energy
    // Note: k1 and k2 are constant for all particles in the curve
    static double ActiveContourInternal(const FittedCurve& curve,
                                        double k1,
                                        double k2);

    // Amalgamation of energy metrics used parameterized by their coefficients.
    // To disable a metric, simply set its coefficient to zero
    static double TotalEnergy(const FittedCurve& curve,
                              double alpha,
                              double k1,
                              double k2,
                              double beta,
                              double delta);

    // Sum of the absolute value of the curvature from curve
    static double AbsCurvatureSum(const FittedCurve& curve);

    // Determine arc length across a window of size 'windowSize' centered at
    // 'index'
    static double LocalWindowedArcLength(const FittedCurve& curve,
                                         int32_t index,
                                         int32_t windowSize);

    // Apply LocalWindowedArcLength across the entire curve
    static double WindowedArcLength(const FittedCurve& curve,
                                    int32_t windowSize);
};
}
}

#endif
