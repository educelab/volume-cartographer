#pragma once

#include "vc/core/types/ITKMesh.hpp"

namespace volcart::meshing
{

/**
 * @brief Apply Laplacian smoothing to a mesh
 *
 * A wrapper around vtkSmoothPolyDataFilter
 */
class LaplacianSmooth
{
public:
    /** @brief Set the input mesh */
    void setInputMesh(const ITKMesh::Pointer& m);

    /** @copydoc iterations() const */
    void setIterations(std::size_t i);
    /** @copydoc relaxationFactor() const */
    void setRelaxationFactor(double f);
    /** @copydoc featureEdgeSmoothing() const */
    void setFeatureEdgeSmoothing(bool b);
    /** @copydoc featureAngle() const */
    void setFeatureAngle(double a);
    /** @copydoc edgeAngle() const */
    void setEdgeAngle(double a);
    /** @copydoc boundarySmoothing() const */
    void setBoundarySmoothing(bool b);

    /** @brief The number of smoothing interations */
    [[nodiscard]] auto iterations() const -> std::size_t;
    /** @brief Relaxation factor for Laplacian smoothing */
    [[nodiscard]] auto relaxationFactor() const -> double;
    /** @brief Smoothing along sharp interior edges */
    [[nodiscard]] auto featureEdgeSmoothing() const -> bool;
    /** @brief Feature angle for sharp edge identification */
    [[nodiscard]] auto featureAngle() const -> double;
    /** @brief Edge angle for sharp edge identification */
    [[nodiscard]] auto edgeAngle() const -> double;
    /** @brief Smoothing vertices on the mesh boundary */
    [[nodiscard]] auto boundarySmoothing() const -> bool;

    /** @brief Compute the smoothed mesh */
    auto compute() -> ITKMesh::Pointer;

    /** @brief Return the smoothed mesh */
    auto getOutputMesh() -> ITKMesh::Pointer;

private:
    /** Input mesh */
    ITKMesh::Pointer input_{nullptr};
    /** Output mesh */
    ITKMesh::Pointer output_{nullptr};
    /** Num iters */
    std::size_t iters_{20};
    /** Relaxation factor */
    double relax_{0.01};
    /** Do feature edge smoothing */
    bool edgeSmooth_{false};
    /** Feature edge angle */
    double featureAngle_{45};
    /** Sharp edge angle */
    double edgeAngle_{15};
    /** Smooth boundary vertices */
    bool boundarySmooth_{true};
};
}  // namespace volcart::meshing