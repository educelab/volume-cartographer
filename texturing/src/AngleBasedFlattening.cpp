#include "vc/texturing/AngleBasedFlattening.hpp"

#include <Eigen/IterativeLinearSolvers>
#include <OpenABF/OpenABF.hpp>

#include "vc/core/util/Logging.hpp"
#include "vc/core/util/MeshMath.hpp"
#include "vc/meshing/ScaleMesh.hpp"

using namespace volcart;
using namespace volcart::meshmath;
using namespace volcart::meshing;
using namespace volcart::texturing;

using MatrixType = Eigen::SparseMatrix<double>;
using HalfEdgeMesh = OpenABF::detail::ABF::Mesh<double>;

// SparseLU
using ABF = OpenABF::ABFPlusPlus<double>;
using LSCM = OpenABF::AngleBasedLSCM<double, HalfEdgeMesh>;
// ConjugateGradient
using CG = Eigen::ConjugateGradient<MatrixType, Eigen::Lower | Eigen::Upper>;
using ABF_CG = OpenABF::ABFPlusPlus<double, HalfEdgeMesh, CG>;
using LSCM_CG = OpenABF::AngleBasedLSCM<double, HalfEdgeMesh, CG>;
// HierarchicalLSCM (uses ConjugateGradient for warm-started hierarchy)
using HLSCM = OpenABF::HierarchicalLSCM<double, HalfEdgeMesh, CG>;

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
    for (const auto cell : *mesh_->GetCells()) {
        hem->insert_face(cell->GetPointIdsContainer());
    }
    hem->update_boundary();

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
            if (solver_ == Solver::SparseLU) {
                ABF::Compute(hem, iters, grad, maxABFIterations_);
            } else if (solver_ == Solver::ConjugateGradient) {
                ABF_CG::Compute(hem, iters, grad, maxABFIterations_);
            }
        } catch (const OpenABF::SolverException& e) {
            Logger()->warn("Failed to solve ABF++. Falling back to LSCM.");
            Logger()->debug("SolverException: {}", e.what());
        }
        Logger()->info(
            "ABF++ Iterations: {} || Final norm: {:.5g}", iters, grad);
    }

    // LSCM
    Logger()->info("Solving {}", useHLSCM_ ? "HierarchicalLSCM" : "LSCM");
    if (useHLSCM_) {
        HLSCM::Compute(hem);
    } else if (solver_ == Solver::SparseLU) {
        LSCM::Compute(hem);
    } else if (solver_ == Solver::ConjugateGradient) {
        LSCM_CG::Compute(hem);
    }

    // Fill output
    // OpenABF flattens to XY, but we want it on XZ
    Logger()->debug("Converting half-edge mesh to output mesh");
    auto flatMesh = DeepCopy(mesh_);
    ITKPoint pt;
    const cv::Vec3d norm{0.0, 1.0, 0.0};
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

void AngleBasedFlattening::setUseHLSCM(bool h) { useHLSCM_ = h; }

auto AngleBasedFlattening::useHLSCM() const -> bool { return useHLSCM_; }

void AngleBasedFlattening::setSolver(const Solver solver) { solver_ = solver; }

auto AngleBasedFlattening::solver() const -> Solver { return solver_; }
