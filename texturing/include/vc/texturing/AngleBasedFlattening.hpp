#pragma once

/** @file */

#include <cstddef>
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
 * Optionally uses ABF++ to calculate optimal interior angles, then flattens
 * the mesh using either HierarchicalLSCM (default) or AngleBasedLSCM.
 *
 * Implementation provided by the
 * [OpenABF library](https://github.com/educelab/OpenABF).
 *
 * @ingroup UV
 */
class AngleBasedFlattening : public FlatteningAlgorithm
{
public:
    /** Solver implementation */
    enum class Solver { SparseLU = 0, ConjugateGradient = 1 };

    /** Default maximum number of ABF iterations */
    static constexpr std::size_t DEFAULT_ITERATIONS{10};

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
     * If `false`, the mesh is flattened using only the LSCM algorithm
     * (standard or hierarchical, depending on @ref useHLSCM()).
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
    /**
     * @brief Whether to use HierarchicalLSCM for parameterization
     *
     * When `true`, uses HierarchicalLSCM with ConjugateGradient for the
     * LSCM step. The @ref solver() setting is ignored for this path.
     * When `false` (default), uses AngleBasedLSCM with the configured
     * @ref solver().
     */
    void setUseHLSCM(bool h);

    /** @brief Whether HierarchicalLSCM is used */
    [[nodiscard]] auto useHLSCM() const -> bool;
    /**@}*/

    /**@{*/
    /**
     * @brief The numerical solver method
     *
     * @note When compiled with OpenMP support, certain Eigen solvers (e.g.
     * ConjugateGradient) are multithreaded. The number of threads used is
     * controlled globally through the OpenMP and/or Eigen interfaces. See
     * [Eigen and
     * multi-threading](https://libeigen.gitlab.io/eigen/docs-nightly/TopicMultiThreading.html)
     * for more information.
     */
    void setSolver(Solver solver);

    /** @copydoc setSolver(Solver) */
    [[nodiscard]] auto solver() const -> Solver;
    /**@}*/

    /**@{*/
    /** @brief Compute the parameterization */
    auto compute() -> ITKMesh::Pointer override;
    /**@}*/

private:
    /** Whether to use ABF minimization */
    bool useABF_{true};
    /** Whether to use HierarchicalLSCM instead of AngleBasedLSCM */
    bool useHLSCM_{false};
    /** Solver method (only used when useHLSCM_ is false) */
    Solver solver_{Solver::SparseLU};
    /** Maximum number of ABF minimization iterations */
    std::size_t maxABFIterations_{DEFAULT_ITERATIONS};
};
}  // namespace volcart::texturing
