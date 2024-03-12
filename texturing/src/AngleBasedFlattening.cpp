#include "vc/texturing/AngleBasedFlattening.hpp"

#include <OpenABF/OpenABF.hpp>

#include "vc/core/util/Logging.hpp"
#include "vc/core/util/MeshMath.hpp"
#include "vc/meshing/ScaleMesh.hpp"

using namespace volcart;
using namespace volcart::meshmath;
using namespace volcart::meshing;
using namespace volcart::texturing;

using ABF = OpenABF::ABFPlusPlus<double>;
using HalfEdgeMesh = ABF::Mesh;
using LSCM = OpenABF::AngleBasedLSCM<double, HalfEdgeMesh>;

AngleBasedFlattening::AngleBasedFlattening(const ITKMesh::Pointer& m)
    : FlatteningAlgorithm(m)
{
}

void AngleBasedFlattening::setUseABF(bool a) { useABF_ = a; }

void AngleBasedFlattening::setABFMaxIterations(std::size_t i)
{
    maxABFIterations_ = i;
}

///// Process //////
auto AngleBasedFlattening::compute() -> ITKMesh::Pointer
{
    // Construct HEM
    auto hem = HalfEdgeMesh::New();

    // Copy the points
    Logger()->debug("Inserting vertices into half-edge mesh");
    OpenABF::Vec3d p;
    for (auto pt = mesh_->GetPoints()->Begin(); pt != mesh_->GetPoints()->End();
         ++pt) {
        p[0] = pt->Value()[0];
        p[1] = pt->Value()[1];
        p[2] = pt->Value()[2];
        hem->insert_vertex(p);
    }

    // Copy the faces
    Logger()->debug("Inserting faces into half-edge mesh");
    OpenABF::Vec<std::size_t, 3> indices;
    for (auto cell = mesh_->GetCells()->Begin();
         cell != mesh_->GetCells()->End(); ++cell) {

        indices[0] = cell.Value()->GetPointIdsContainer()[0];
        indices[1] = cell.Value()->GetPointIdsContainer()[1];
        indices[2] = cell.Value()->GetPointIdsContainer()[2];
        hem->insert_face(indices);
    }

    // Sanity check
    Logger()->debug("Checking that half-edge mesh is manifold");
    if (not OpenABF::IsManifold(hem)) {
        throw std::runtime_error("Input mesh is not manifold.");
    }

    // ABF
    if (useABF_) {
        Logger()->info("Solving ABF++");
        std::size_t iters{0};
        double grad{0};
        try {
            ABF::Compute(hem, iters, grad, maxABFIterations_);
        } catch (const OpenABF::SolverException& e) {
            Logger()->warn("Failed to solve ABF++. Falling back to LSCM.");
            Logger()->debug("SolverException: {}", e.what());
        }
        Logger()->info(
            "ABF++ Iterations: {} || Final norm: {:.5g}", iters, grad);
    }

    // LSCM
    Logger()->info("Solving LSCM");
    LSCM::Compute(hem);

    // Fill output
    // OpenABF flattens to XY, but we want it on XZ
    Logger()->debug("Converting half-edge mesh to output mesh");
    auto flatMesh = ITKMesh::New();
    DeepCopy(mesh_, flatMesh);
    ITKPoint pt;
    cv::Vec3d norm{0.0, 1.0, 0.0};
    for (const auto& v : hem->vertices()) {
        pt[0] = v->pos[0];
        pt[1] = 0.0;
        pt[2] = v->pos[1];
        flatMesh->SetPoint(v->idx, pt);
        flatMesh->SetPointData(v->idx, norm.val);
    }

    // Scale mesh surface area to same as original
    auto scale = std::sqrt(SurfaceArea(mesh_) / SurfaceArea(flatMesh));
    Logger()->debug("Scaling output mesh by scale factor {:.5g}", scale);
    output_ = ITKMesh::New();
    ScaleMesh(flatMesh, output_, scale);

    return output_;
}

auto AngleBasedFlattening::useABF() const -> bool { return useABF_; }

auto AngleBasedFlattening::abfMaxIterations() const -> std::size_t
{
    return maxABFIterations_;
}
