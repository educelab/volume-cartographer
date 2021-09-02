#pragma once

/** @file */

#include "vc/texturing/TexturingAlgorithm.hpp"

#include "vc/core/neighborhood/NeighborhoodGenerator.hpp"

namespace volcart::texturing
{
/**
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
    static auto New() -> Pointer;

    /** Default destructor */
    ~IntegralTexture() override = default;
    /**@}*/

    /**@{*/
    /**
     * @brief Set the Neighborhood generator
     *
     * This class supports generators of dimension >= 1
     */
    void setGenerator(NeighborhoodGenerator::Pointer g);

    /**
     * @brief When enabled, clamp neighborhood intensities to the value
     * specified by setClampMax()
     *
     * Note: Clamping is performed prior to integration of the neighborhood
     */
    void setClampValuesToMax(bool b);

    /** @copydoc setClampValuesToMax(bool) */
    [[nodiscard]] auto clampValuesToMax() const -> bool;

    /**
     * @brief The maximum intensity value allowed in neighborhood prior to
     * integration
     *
     * Ignored if setClampValuesToMax() is set to `false`
     *
     * Default: std::numeric_limits<uint16_t>::max()
     */
    void setClampMax(uint16_t m);

    /** @copydoc setClampMax(uint16_t) */
    [[nodiscard]] auto clampMax() const -> uint16_t;

    /**
     * @brief Set the weighting method
     *
     * Default: None
     */
    void setWeightMethod(WeightMethod w);

    /** @copydoc setWeightMethod(WeightMethod) */
    [[nodiscard]] auto weightMethod() const -> WeightMethod;

    /**
     * @brief Set the linear weight direction
     *
     * Default: Positive
     */
    void setLinearWeightDirection(LinearWeightDirection w);

    /** @copydoc setLinearWeightDirection(LinearWeightDirection) */
    [[nodiscard]] auto linearWeightDirection() const -> LinearWeightDirection;

    /**
     * @brief Set the weighting exponent used by Exponential Difference
     * weighting
     */
    void setExponentialDiffExponent(int e);

    /** @copydoc setExponentialDiffExponent(int) */
    [[nodiscard]] auto exponentialDiffExponent() const -> int;

    /**
     * @brief Set the method used to calculate the Exponential Difference base
     * value
     *
     * Default: Mean
     */
    void setExponentialDiffBaseMethod(ExpoDiffBaseMethod m);

    /** @copydoc setExponentialDiffBaseMethod(ExpoDiffBaseMethod) */
    [[nodiscard]] auto exponentialDiffBaseMethod() const -> ExpoDiffBaseMethod;

    /** @brief Set the base value for Exponential Difference weighting */
    void setExponentialDiffBaseValue(double b);

    /** @copydoc setExponentialDiffBaseValue(double) */
    [[nodiscard]] auto exponentialDiffBaseValue() const -> double;

    /**
     * @brief When enabled, do not integrate intensity values below the base
     * value
     *
     * For exponential difference weighting only.
     *
     * Default: True
     */
    void setExponentialDiffSuppressBelowBase(bool b);

    /** @copydoc setExponentialDiffSuppressBelowBase(bool) */
    [[nodiscard]] auto exponentialDiffSuppressBelowBase() const -> bool;
    /**@}*/

    /**@{*/
    /** @brief Compute the Texture */
    auto compute() -> Texture override;
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
    auto apply_weights_(NDArray<double>& n) -> NDArray<double>;

    /** Linear weighting direction */
    LinearWeightDirection linearWeight_{LinearWeightDirection::Positive};

    /** Linear weights vector */
    NDArray<double> linearWeights_{1};

    /** Setup the linear weights vector */
    void setup_linear_weights_();

    /** Apply the linear weights vector to a neighborhood */
    auto apply_linear_weights_(NDArray<double>& n) -> NDArray<double>;

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
    auto expodiff_intersection_pts_() -> std::vector<uint16_t>;

    /** Calculate the mean base value */
    auto expodiff_mean_base_() -> double;

    /** Calculate the mode base value */
    auto expodiff_mode_base_() -> double;

    /** Apply the expo diff weights to a neighborhood */
    auto apply_expodiff_weights_(NDArray<double>& n) const -> NDArray<double>;
};

}  // namespace volcart::texturing
