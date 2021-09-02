#include "vc/meshing/LaplacianSmooth.hpp"

#include <vtkSmoothPolyDataFilter.h>

#include "vc/meshing/CalculateNormals.hpp"
#include "vc/meshing/ITK2VTK.hpp"

using namespace volcart;
using namespace volcart::meshing;

void LaplacianSmooth::setInputMesh(const ITKMesh::Pointer& m) { input_ = m; }
void LaplacianSmooth::setIterations(std::size_t i) { iters_ = i; }
void LaplacianSmooth::setRelaxationFactor(double f) { relax_ = f; }
void LaplacianSmooth::setFeatureEdgeSmoothing(bool b) { edgeSmooth_ = b; }
void LaplacianSmooth::setFeatureAngle(double a) { featureAngle_ = a; }
void LaplacianSmooth::setEdgeAngle(double a) { edgeAngle_ = a; }
void LaplacianSmooth::setBoundarySmoothing(bool b) { boundarySmooth_ = b; }

auto LaplacianSmooth::iterations() const -> std::size_t { return iters_; }
auto LaplacianSmooth::relaxationFactor() const -> double { return relax_; }
auto LaplacianSmooth::featureEdgeSmoothing() const -> bool
{
    return edgeSmooth_;
}
auto LaplacianSmooth::featureAngle() const -> double { return featureAngle_; }
auto LaplacianSmooth::edgeAngle() const -> double { return edgeAngle_; }
auto LaplacianSmooth::boundarySmoothing() const -> bool
{
    return boundarySmooth_;
}

auto LaplacianSmooth::compute() -> ITKMesh::Pointer
{
    auto vtkMesh = ITK2VTK(input_);
    auto smoother = vtkSmartPointer<vtkSmoothPolyDataFilter>::New();
    smoother->SetInputData(vtkMesh);
    smoother->SetNumberOfIterations(static_cast<int>(iters_));
    smoother->SetRelaxationFactor(relax_);
    smoother->SetFeatureEdgeSmoothing(static_cast<vtkTypeBool>(edgeSmooth_));
    smoother->SetFeatureAngle(featureAngle_);
    smoother->SetEdgeAngle(edgeAngle_);
    smoother->SetBoundarySmoothing(static_cast<vtkTypeBool>(boundarySmooth_));
    smoother->Update();
    output_ = VTK2ITK(smoother->GetOutput());

    // Recalculate the normals on the mesh
    CalculateNormals normals;
    normals.setMesh(output_);
    output_ = normals.compute();

    return output_;
}

auto LaplacianSmooth::getOutputMesh() -> ITKMesh::Pointer { return output_; }
