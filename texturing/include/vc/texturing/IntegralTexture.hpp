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
    /**
     * @brief Weight option
     *
     * Setting the weight option applies a linear weight factor to the intensity
     * values of the neighborhood. The options are named according to which
     * values along a point's surface normal are favored.
     *
     * The weight factors for the options are as follows:
     * - Positive: Most Positive: 1.0, Least Positive: 0.0
     * - Negative: Most Positive: 0.0, Least Positive: 1.0
     * - None: Most Positive: 1.0, Least Positive: 1.0
     */
    enum class Weight { Positive, Negative, None };

    /**@{*/
    /**
     * @brief Set the weight option
     *
     * Default: None
     */
    void setWeight(Weight w) { weightType_ = w; }
    /**@}*/

    /**@{*/
    /** @brief Compute the Texture */
    Texture compute();
    /**@}*/
private:
    /** Weighting option */
    Weight weightType_{Weight::None};
    /** Current weight value */
    double currentWeight_;
    /** Current weight increment value */
    double weightStep_;
    /** Setup the weight values */
    void setup_weights_(size_t s);
};

}  // texturing
}  // volcart
