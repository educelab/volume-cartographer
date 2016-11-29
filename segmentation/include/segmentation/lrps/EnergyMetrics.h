#pragma once
#include "segmentation/lrps/FittedCurve.h"

namespace volcart
{
namespace segmentation
{
/**
 * @class EnergyMetrics
 *
 * @brief Calcuate energy to see how particles move
 *
 * Use various algorithms and formulas to calculate how particles
 * move through the volume
 *
 * @ingroup lrps
 */
class EnergyMetrics
{
public:
    /**
     * @brief Calculates the active contour internal energy
     *
     * Calculates the internal energy of fitted curved based on the
     * first and second derivatives of that curve squared. The result
     * for the first curve is then multiplied by k1 and the result
     * for the second curive is multiplied by k2
     * Note: k1 and k2 are constant for all particles in the curve.
     *
     * @see https://en.wikipedia.org/wiki/Active_contour_model#Internal_energy
     */
    static double ActiveContourInternal(
        const FittedCurve& curve, double k1, double k2);

    /**
     * @brief Amalgamation of energy metrics used parameterized by their
     * coefficients
     *
     * Uses the internal energy, the arc length and the sum of the curvature to
     * calculate the total amount of energy in the system so that when
     * segmenting the rules of energy can be followed.
     * Note: To disable a metric, simply set its coefficient to zero
     *
     */
    static double TotalEnergy(
        const FittedCurve& curve,
        double alpha,
        double k1,
        double k2,
        double beta,
        double delta);

    /**
     * @brief Sum of the absolute value of the curvature from curve
     */
    static double AbsCurvatureSum(const FittedCurve& curve);

    /**
     * @brief arc length across a window of size 'windowSize' centered at
     * 'index'
     */
    static double LocalWindowedArcLength(
        const FittedCurve& curve, int32_t index, int32_t windowSize);

    /**
     * @brief Apply LocalWindowedArcLength across the entire curve
     */
    static double WindowedArcLength(
        const FittedCurve& curve, int32_t windowSize);
};
}
}
