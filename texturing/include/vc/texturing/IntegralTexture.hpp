#pragma once

/** @file */

#include "vc/texturing/TexturingAlgorithm.hpp"

#include "vc/core/neighborhood/NeighborhoodGenerator.hpp"

namespace volcart::texturing
{
/**
 * @class IntegralTexture
 * @author Seth Parker
 * @date 11/24/2016
 *
 * @brief Generate a Texture by taking the discrete integral (summation) of the
 * neighborhood adjacent to a point
 *
 * @ingroup Texture
 */
class IntegralTexture : public TexturingAlgorithm
{
public:
    /**
     * @brief Weighting Methods
     *
     * The method by which neighborhoods are weighted prior to integration:
     *   - None: No weighting is performed.
     *   - Linear: Values are linearly weighted based on their position in the
     *    neighborhood. See LinearWeightDirection for more details.
     *   - ExpoDiff: Intensity values are exponentially weighted based on their
     *    difference from a base intensity value.
     */
    enum class WeightMethod { None = 0, Linear, ExpoDiff };

    /**
     * @brief Linear weight direction
     *
     * Setting the weight option applies a linear weight factor to the intensity
     * values of the neighborhood. The options are named according to which
     * values along a point's surface normal are favored.
     *
     * The weight factors for the options are as follows:
     *   - Positive: Most Positive: 1.0, Least Positive: 0.0
     *   - Negative: Most Positive: 0.0, Least Positive: 1.0
     */
    enum class LinearWeightDirection { Positive = 0, Negative };

    /**
     * @brief Exponential difference base calculation method
     *
     * The method by which the base value is calculated for Exponential
     * Difference weighting:
     *   - Mean: The average intensity value on the surface of the mesh
     *   - Mode: The most frequent intensity value on the surface of the mesh
     *   - Manual: The value specified by setExponentialDiffBaseValue()
     */
    enum class ExpoDiffBaseMethod { Mean = 0, Mode, Manual };

    /**@{*/
    /** Pointer type */
    using Pointer = std::shared_ptr<IntegralTexture>;

    /** Make shared pointer */
    static Pointer New() { return std::make_shared<IntegralTexture>(); }

    /** Default destructor */
    ~IntegralTexture() override = default;
    /**@}*/

    /**@{*/
    /**
     * @brief Set the Neighborhood generator
     *
     * This class supports generators of dimension >= 1
     */
    void setGenerator(NeighborhoodGenerator::Pointer g) { gen_ = std::move(g); }

    /**
     * @brief When enabled, clamp neighborhood intensities to the value
     * specified by setClampMax()
     *
     * Note: Clamping is performed prior to integration of the neighborhood
     */
    void setClampValuesToMax(bool b) { clampToMax_ = b; }

    /**
     * @brief The maximum intensity value allowed in neighborhood prior to
     * integration
     *
     * Ignored if setClampValuesToMax() is set to `false`
     *
     * Default: std::numeric_limits<uint16_t>::max()
     */
    void setClampMax(uint16_t m) { clampMax_ = m; }

    /**
     * @brief Set the weighting method
     *
     * Default: None
     */
    void setWeightMethod(WeightMethod w) { weight_ = w; }

    /**
     * @brief Set the linear weight direction
     *
     * Default: Positive
     */
    void setLinearWeightDirection(LinearWeightDirection w)
    {
        linearWeight_ = w;
    }

    /**
     * @brief Set the weighting exponent used by Exponential Difference
     * weighting
     */
    void setExponentialDiffExponent(int e) { expoDiffExponent_ = e; }

    /**
     * @brief Set the method used to calculate the Exponential Difference base
     * value
     *
     * Default: Mean
     */
    void setExponentialDiffBaseMethod(ExpoDiffBaseMethod m)
    {
        expoDiffBaseMethod_ = m;
    }

    /** @brief Set the base value for Exponential Difference weighting */
    void setExponentialDiffBaseValue(double b) { expoDiffManualBase_ = b; }

    /**
     * @brief When enabled, do not integrate intensity values below the base
     * value
     *
     * For exponential difference weighting only.
     *
     * Default: True
     */
    void setExponentialDiffSuppressBelowBase(bool b) { suppressBelowBase_ = b; }
    /**@}*/

    /**@{*/
    /** @brief Compute the Texture */
    Texture compute() override;
    /**@}*/

private:
    /** Neighborhood generator */
    NeighborhoodGenerator::Pointer gen_;

    /** Enable/Disable clamping to maximum value */
    bool clampToMax_{false};

    /** Maximum allowed value in neighborhood when clamping is enabled */
    uint16_t clampMax_{std::numeric_limits<uint16_t>::max()};

    /** Selected Weighting method */
    WeightMethod weight_{WeightMethod::None};

    /** Setup the selected weighting method */
    void setup_weights_();

    /** Apply the selected weighting method */
    NDArray<double> apply_weights_(NDArray<double>& n);

    /** Linear weighting direction */
    LinearWeightDirection linearWeight_{LinearWeightDirection::Positive};

    /** Linear weights vector */
    NDArray<double> linearWeights_{1};

    /** Setup the linear weights vector */
    void setup_linear_weights_();

    /** Apply the linear weights vector to a neighborhood */
    NDArray<double> apply_linear_weights_(NDArray<double>& n);

    /** Exponential diff exponent */
    int expoDiffExponent_{2};

    /** Exponential diff base calculation method */
    ExpoDiffBaseMethod expoDiffBaseMethod_{ExpoDiffBaseMethod::Mean};

    /** Manually specified exponential diff base */
    double expoDiffManualBase_{0};

    /** Exponential diff base value that is used */
    double expoDiffBase_{0};

    /** Whether or not to ignore values below the base value */
    bool suppressBelowBase_{true};

    /** Setup the expo diff weights */
    void setup_expodiff_weights_();

    /** Get the list of intensities on the surface of the mesh */
    std::vector<uint16_t> expodiff_intersection_pts_();

    /** Calculate the mean base value */
    double expodiff_mean_base_();

    /** Calculate the mode base value */
    double expodiff_mode_base_();

    /** Apply the expo diff weights to a neighborhood */
    NDArray<double> apply_expodiff_weights_(NDArray<double>& n);
};

}  // namespace volcart::texturing
