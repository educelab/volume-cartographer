#pragma once

/** @file */

#include "vc/segmentation/lrps/FittedCurve.hpp"

namespace volcart::segmentation
{
/**
 * @class EnergyMetrics
 * @brief A collection of energy metrics for evaluating a FittedCurve
 *
 * @ingroup lrps
 */
class EnergyMetrics
{
public:
    /**
     * @brief Calculate the active contour internal energy of a FittedCurve
     *
     * k1 and k2 are user-defined weights on the first and second derivatives
     * of the curve.
     *
     * From Wikipedia: "These control the internal energy function's
     * sensitivity to the amount of stretch in the snake and the amount of
     * curvature in the snake, respectively, and thereby control the number of
     * constraints on the shape of the snake. In practice, a large weight k1
     * for the continuity term penalizes changes in distances between points
     * in the contour. A large weight k2 for the smoothness term penalizes
     * oscillations in the contour and will cause the contour to act as a
     * thin plate."
     *
     * @see https://en.wikipedia.org/wiki/Active_contour_model#Internal_energy
     *
     * @param curve Input curve
     * @param k1 Stretch weight factor
     * @param k2 Curvature weight factor
     *
     */
    static double ActiveContourInternal(
        const FittedCurve& curve, double k1, double k2);

    /**
     * @brief Combinatorial energy metric for a FittedCurve
     *
     * Calculates a weighted sum of ActiveContourInternal(), AbsCurvatureSum(),
     * and WindowedArcLength().
     * \f[
       E_t = \alpha ACI(k1, k2) + \beta ACS() + \delta WAL(3);
     * \f]
     * To disable a metric, simply set its coefficient to zero.
     *
     * @param curve Input curve
     * @param alpha ActiveContourInternal() total weight factor
     * @param k1 ActiveContourInternal() stretch weight factor
     * @param k2 ActiveContourInternal() curvature weight factor
     * @param beta AbsCurvatureSum() total weight factor
     * @param delta WindowedArcLength() total weight factor
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
     * @brief Sum of the absolute value of local curvature at each point along
     * the curve
     * @param curve Input curve
     */
    static double AbsCurvatureSum(const FittedCurve& curve);

    /**
     * @brief Calculate the arc length of the curve within a window centered
     * at a point along the curve
     * @param curve Input curve
     * @param index Center point of calculation window
     * @param windowSize Size of calculation window
     */
    static double LocalWindowedArcLength(
        const FittedCurve& curve, int index, int windowSize);

    /**
     * @brief Average LocalWindowedArcLength() across the entire curve
     * @param curve Input curve
     * @param windowSize Size of calculation window
     */
    static double WindowedArcLength(const FittedCurve& curve, int windowSize);
};
}  // namespace volcart::segmentation
