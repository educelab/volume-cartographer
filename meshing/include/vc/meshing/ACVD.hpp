#pragma once

/** @file */

#include "vc/core/types/ITKMesh.hpp"

namespace volcart
{
namespace meshing
{
/**
 * @brief Mesh resampling using Approximated Centroidal Voronoi Diagrams.
 *
 * This is a wrapper around the ACVD implementation found in the ACVD.cxx
 * example of <a href="https://github.com/valette/ACVD">ACVD</a>. This
 * implements the iterative process discussed in:
 *      Valette, Sebastien, and Jean-Marc Chassery. "Approximated centroidal
 *      voronoi diagrams for uniform polygonal mesh coarsening." Computer
 *      Graphics Forum. Vol. 23. No. 3. Blackwell Publishing, Inc, 2004.
 *
 * Iteratively loops over the mesh until the approximated centroidal voronoi
 * diagrams for the mesh are approximately equivalent in area. It then takes
 * the point on the mesh that is nearest to the centroid of each diagram as a
 * new vertex in the resampled output mesh.
 *
 * This class provides both the isotropic and anisotropic versions of the
 * algorithm.
 *
 * @ingroup Meshing
 */
class ACVD
{
public:
    /** @brief Isotropy modes */
    enum class Mode { Isotropic, Anisotropic };

    ACVD() = default;

    /** @brief Set the input mesh */
    void setInputMesh(ITKMesh::Pointer input);

    /** @brief Set the isotropy mode */
    void setMode(Mode m);

    /**
     * @brief The number of voronoi clusters to use for remeshing
     *
     * Because this algorithm uses the center of voronoi clusters as the
     * vertices of the output mesh, this parameter sets the approximate
     * number of vertices desired in the output mesh.
     *
     * If set to 0 (default), the number of vertices in the input mesh will
     * be used.
     */
    void setNumberOfClusters(size_t n);

    /**
     * @brief Set the gradation (curvature) constraint
     *
     * If set to 0 (default), uniform clustering will be performed. Higher
     * values will produce a clustering which favors regions of high curvature.
     */
    void setGradation(double g);

    /**
     * @brief Set the subsample threshold
     *
     * Input mesh will be iteratively divided until the subsample ratio is
     * above the subsampling threshold. In practice, higher values produce
     * better results, but require more subdivision.
     */
    void setSubsampleThreshold(size_t t);

    /**
     * @brief Set the quadrics optimization level
     *
     * For values greater than 0, refine the resampled vertices to minimize the
     * quadric error between the input and output mesh. Only use if mode is
     * `Mode::Isotropic`. Default value: 1
     */
    void setQuadricsOptimizationLevel(size_t l);

    /** @brief Compute the resampled mesh */
    ITKMesh::Pointer compute();

    /** @brief Get the output mesh */
    ITKMesh::Pointer getOutputMesh() const;

private:
    /** Input mesh */
    ITKMesh::Pointer inputMesh_;
    /** Output mesh */
    ITKMesh::Pointer outputMesh_;

    /** Compute ACVD isotropic */
    void compute_isotropic_();
    /** Compute ACVD anisotropic */
    void compute_anisotropic_();
    /** Isotropy mode */
    Mode mode_{Mode::Isotropic};
    /** Number of clusters */
    size_t clusters_{0};
    /** Gradation */
    double gradation_{0};
    /** Subsample Threshold */
    size_t subsampleThreshold_{10};
    /** Quadrics optimization level */
    size_t quadricsOptLevel_{1};
};
}  // namespace meshing
}  // namespace volcart
