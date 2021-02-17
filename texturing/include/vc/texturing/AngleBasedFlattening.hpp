#pragma once

/** @file */

#include <memory>

#include <opencv2/core.hpp>

#include "vc/core/types/ITKMesh.hpp"
#include "vc/core/types/UVMap.hpp"
#include "vc/texturing/FlatteningAlgorithm.hpp"

namespace volcart::texturing
{
/**
 * @class AngleBasedFlattening
 *
 * @ingroup UV
 */
class AngleBasedFlattening : public FlatteningAlgorithm
{
public:
    /** Default maximum number of ABF iterations */
    static const std::size_t DEFAULT_ITERATIONS{10};

    /** Pointer */
    using Pointer = std::shared_ptr<AngleBasedFlattening>;

    /**@{*/
    /** @brief Default constructor */
    AngleBasedFlattening() = default;

    /** @brief Construct and set the input mesh */
    explicit AngleBasedFlattening(const ITKMesh::Pointer& m);

    /** Make a new shared instance */
    template <typename... Args>
    static Pointer New(Args... args)
    {
        return std::make_shared<AngleBasedFlattening>(
            std::forward<Args>(args)...);
    }

    /** Default destructor */
    ~AngleBasedFlattening() override = default;
    /**@}*/

    /**@{*/
    /**
     * @brief Set whether to perform Angle-based flattening computation
     *
     * Setting this value to `false` results in a mesh that is flattened using
     * only the LSCM algorithm.
     */
    void setUseABF(bool a);

    /** @brief Set the max number of ABF minimization iterations */
    void setABFMaxIterations(std::size_t i);
    /**@}*/

    /**@{*/
    /** @brief Compute the parameterization */
    ITKMesh::Pointer compute() override;
    /**@}*/

private:
    /** Whether to use ABF minimization */
    bool useABF_{true};
    /** Maximum number of ABF minimization iterations */
    std::size_t maxABFIterations_{DEFAULT_ITERATIONS};
};
}  // namespace volcart::texturing
