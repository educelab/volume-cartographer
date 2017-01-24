#pragma once
#include "segmentation/lrps/FittedCurve.hpp"

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
     * @param curve Curve whose Internal Energy you wish to calculate
     * @param k1 Modifier for the first derivative
     * @param k2 Modifier for the second derivative
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
     * @param curve Curve whose total energy you wish to calculate
     * @param alpha Used as a multiplier
     * @param k1 Used to find internal energy
     * @param k2 Used to find internal energy
     * @param beta Used as a multiplier
     * @param delta Used as a multiplier
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
     * @param curve Curve who's curvature sum you wish to find
     */
    static double AbsCurvatureSum(const FittedCurve& curve);

    /**
     * @brief arc length across a window of size 'windowSize' centered at
     * 'index'
     * @param curve Curve who's arc length you wish to find
     * @param index Center of arc length
     * @param windowSize Size of window where curve will be displayed
     */
    static double LocalWindowedArcLength(
        const FittedCurve& curve, int index, int windowSize);

    /**
     * @brief Apply LocalWindowedArcLength across the entire curve
     * @param windowSize Size of window where curve will be displayed
     * @param curve Curve you wish to display
     */
    static double WindowedArcLength(const FittedCurve& curve, int windowSize);
};
}
}
