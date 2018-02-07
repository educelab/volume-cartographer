#pragma once

#include "vc/texturing/TexturingAlgorithmBaseClass.hpp"

namespace volcart
{
namespace texturing
{
/**
 * @class IntegralTexture
 * @author Seth Parker
 * @date 11/24/2016
 *
 * @brief Generate a Texture by taking the discrete integral (summation) of the
 * linear neighborhood adjacent to a point
 *
 * @ingroup Texture
 */
class IntegralTexture : public TexturingAlgorithmBaseClass
{
public:
    /** Default destructor */
    ~IntegralTexture() override = default;

    /** @brief Weighting options */
    enum class WeightType { None = 0, Linear, ExpoDiff };

    /**
     * @brief Linear weight direction
     *
     * Setting the weight option applies a linear weight factor to the intensity
     * values of the neighborhood. The options are named according to which
     * values along a point's surface normal are favored.
     *
     * The weight factors for the options are as follows:
     * - Positive: Most Positive: 1.0, Least Positive: 0.0
     * - Negative: Most Positive: 0.0, Least Positive: 1.0
     */
    enum class LinearWeightDirection { Positive = 0, Negative };

    enum class ExpoDiffBaseMethod { Mean = 0, Mode, Manual };

    /**@{*/
    void setClampValuesToMax(bool b) { clamp_to_max_ = b; }
    void setClampMax(uint16_t m) { clamp_max_ = m; }

    /** @brief Set the weighting type */
    void setWeightType(WeightType w) { weight_ = w; }

    /**
     * @brief Set the linear weight direction
     *
     * Default: Positive
     */
    void setLinearWeightDirection(LinearWeightDirection w)
    {
        linearWeight_ = w;
    }

    /** @brief Set the exponent used by Exponential Difference weighting */
    void setExponentialDiffExponent(int e) { expoDiffExponent_ = e; }

    void setExponentialDiffBaseMethod(ExpoDiffBaseMethod m)
    {
        expoDiffBaseMethod_ = m;
    }

    void setExponentialDiffBase(double b) { expoDiffManualBase_ = b; }

    void setMin() {}
    void setMax() {}
    /**@}*/

    /**@{*/
    /** @brief Compute the Texture */
    Texture compute() override;
    /**@}*/
private:
    /** Clamp */
    bool clamp_to_max_{false};
    uint16_t clamp_max_{std::numeric_limits<uint16_t>::max()};

    /** Selected Weighting option */
    WeightType weight_{WeightType::None};

    /** Setup the weight values */
    void setup_weights_();

    /** Apply weights to the neighborhood */
    std::vector<double> apply_weights_(std::vector<double>& v);

    /** Linear Weighting option */
    LinearWeightDirection linearWeight_{LinearWeightDirection::Positive};
    std::vector<double> linearWeights_;
    void setup_linear_weights_();
    std::vector<double> apply_linear_weights_(std::vector<double>& v);

    /** Exponential Diff option */
    int expoDiffExponent_{2};
    ExpoDiffBaseMethod expoDiffBaseMethod_{ExpoDiffBaseMethod::Mean};
    double expoDiffManualBase_{0};
    double expoDiffBase_{0};
    bool suppressBelowBase_{true};
    void setup_expodiff_weights_();
    std::vector<uint16_t> expodiff_intersection_pts();
    double expodiff_mean_base_();
    double expodiff_mode_base_();
    std::vector<double> apply_expodiff_weights_(std::vector<double>& v);
};

}  // texturing
}  // volcart
