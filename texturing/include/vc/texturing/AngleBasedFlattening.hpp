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
 * @brief Parameterize a mesh using ABF++
 *
 * First, uses ABF++ to calculate the optimal, interior angles of the flattened
 * mesh. Then uses an Angle-based formulation of Least Squares Conformal Maps to
 * convert this angle-optimized parameterization into a full mesh
 * parameterization.
 *
 * Implementation provided by the
 * [OpenABF library](https://gitlab.com/educelab/OpenABF).
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
    static auto New(Args... args) -> Pointer
    {
        return std::make_shared<AngleBasedFlattening>(
            std::forward<Args>(args)...);
    }

    /** Default destructor */
    ~AngleBasedFlattening() override = default;
    /**@}*/

    /**@{*/
    /**
     * @brief Whether to perform Angle-based flattening computation
     *
     * If `false`, the mesh is flattened using only the Angle-based LSCM
     * algorithm.
     */
    void setUseABF(bool a);

    /** @brief Whether Angle-based flattening is performed */
    [[nodiscard]] auto useABF() const -> bool;

    /** @brief The max number of ABF minimization iterations */
    void setABFMaxIterations(std::size_t i);

    /** @copydoc setABFMaxIterations(std::size_t) */
    [[nodiscard]] auto abfMaxIterations() const -> std::size_t;
    /**@}*/

    /**@{*/
    /** @brief Compute the parameterization */
    auto compute() -> ITKMesh::Pointer override;
    /**@}*/

private:
    /** Whether to use ABF minimization */
    bool useABF_{true};
    /** Maximum number of ABF minimization iterations */
    std::size_t maxABFIterations_{DEFAULT_ITERATIONS};
};
}  // namespace volcart::texturing
