// Angle-based Flattening (abf)
// Created by Seth Parker on 6/9/16.
//

#include <cmath>

#include "external/eigen_capi.h"
#include "meshing/DeepCopy.hpp"
#include "texturing/AngleBasedFlattening.hpp"

using namespace volcart;
using namespace volcart::texturing;

///// Get Output /////
// Get output as mesh
ITKMesh::Pointer AngleBasedFlattening::getMesh()
{
    auto output = ITKMesh::New();
    volcart::meshing::DeepCopy(mesh_, output);

    // Update the point positions
    ITKPoint p;
    for (auto it = heMesh_.getVertsBegin(); it != heMesh_.getVertsEnd(); ++it) {
        p[0] = (*it)->uv[0];
        p[1] = 0;
        p[2] = (*it)->uv[1];
        output->SetPoint((*it)->id, p);
    }

    // To-do: #189
    return output;
}

// Get UV Map created from flattened object
volcart::UVMap AngleBasedFlattening::getUVMap()
{
    // Setup uvMap
    volcart::UVMap uvMap;
    uvMap.origin(VC_ORIGIN_BOTTOM_LEFT);

    double uMin = std::numeric_limits<double>::max();
    double uMax = std::numeric_limits<double>::min();
    double vMin = std::numeric_limits<double>::max();
    double vMax = std::numeric_limits<double>::min();

    // Get the min and max values for u & v respectively
    for (auto it = heMesh_.getVertsBegin(); it != heMesh_.getVertsEnd(); ++it) {
        auto vert = (*it);
        if (vert->uv[0] < uMin) {
            uMin = vert->uv[0];
        }
        if (vert->uv[0] > uMax) {
            uMax = vert->uv[0];
        }
        if (vert->uv[1] < vMin) {
            vMin = vert->uv[1];
        }
        if (vert->uv[1] > vMax) {
            vMax = vert->uv[1];
        }
    }

    // Scale width and height back to volume coordinates
    double aspectWidth = std::abs(uMax - uMin);
    double aspectHeight = std::abs(vMax - vMin);
    uvMap.ratio(aspectWidth, aspectHeight);

    // Calculate uv coordinates
    cv::Vec2d uv;
    for (auto it = heMesh_.getVertsBegin(); it != heMesh_.getVertsEnd(); ++it) {
        auto vert = (*it);
        uv[0] = (vert->uv[0] - uMin) / (uMax - uMin);
        uv[1] = (vert->uv[1] - vMin) / (vMax - vMin);

        // Add the uv coordinates into our map at the point index specified
        uvMap.set(vert->id, uv);
    }

    uvMap.origin(VC_ORIGIN_TOP_LEFT);
    return uvMap;
}

///// Process //////
void AngleBasedFlattening::compute()
{
    // Construct the mesh and get the angles
    fill_half_edge_mesh_();

    if (useABF_) {
        scale_();
        solve_abf_();
    }

    // Solve the system via LSCM
    solve_lscm_();
}

///// Setup /////
// Fill the half edge mesh using the currently assigned mesh
void AngleBasedFlattening::fill_half_edge_mesh_()
{
    heMesh_.clear();

    ///// Vertices /////
    for (auto point = mesh_->GetPoints()->Begin();
         point != mesh_->GetPoints()->End(); ++point) {
        heMesh_.addVert(
            point->Value()[0], point->Value()[1], point->Value()[2]);
    }

    ///// Faces /////
    for (auto cell = mesh_->GetCells()->Begin();
         cell != mesh_->GetCells()->End(); ++cell) {
        // Collect the point id's
        std::array<uint64_t, 3> vIds;
        std::copy(
            cell.Value()->PointIdsBegin(), cell.Value()->PointIdsEnd(),
            std::begin(vIds));

        heMesh_.addFace(vIds[0], vIds[1], vIds[2]);
    }
    j2dt_ = cv::Mat(static_cast<int>(heMesh_.getNumberOfEdges()), 3, CV_64F);
    limit_ = (heMesh_.getNumberOfFaces() > 100) ? 1.0f : 0.001f;

    ///// Connectivity /////
    heMesh_.constructConnectedness();

    ///// Generate unique ids just for interior points /////
    bInterior_ =
        std::vector<double>(heMesh_.getNumberOfInteriorPoints() * 2, 0);
    uint64_t interiorId = 0;
    for (auto v = heMesh_.getVert(0); v; v = v->nextLink) {
        if (v->interior()) {
            interior_[v->id] = interiorId++;
        }
    }
}

///// ABF /////
// Scale angles to prevent degenerate cases
void AngleBasedFlattening::scale_()
{
    for (auto v = heMesh_.getVert(0); v; v = v->nextLink) {
        if (v->interior()) {

            double anglesum = 0.0;

            auto e = v->edge;
            do {
                anglesum += e->angle->beta;
                e = e->next->next->pair;
            } while (e && (e != v->edge));

            // Update optimal angle
            if (anglesum == 0.0f) {
                v->edge->angle->phi = 0.0;
            } else {
                v->edge->angle->phi =
                    v->edge->angle->beta * 2.0f * M_PI / anglesum;
            }
        }

        // Re-calculate weight
        v->edge->angle->weight =
            1 / (v->edge->angle->phi * v->edge->angle->phi);
    }
}

// Angle minimization loop
void AngleBasedFlattening::solve_abf_()
{
    compute_sines_();

    double norm = 1e10;  // Magnitude of the error vector?
    int i = 0;
    for (; i < maxABFIterations_; ++i) {
        norm = compute_gradient_();

        // Break if we're below the error limit
        if (norm < limit_) {
            break;
        }

        // Attempt to invert the matrix and solve
        if (!invert_matrix_()) {
            std::cerr
                << "volcart::texturing::abf: ABF failed to invert matrix after "
                << i + 1 << " iterations. Falling back to LSCM." << std::endl;
            throw std::runtime_error("ABF failed to invert matrix");
        }

        // Update the HEM with their new sine/cosine values
        compute_sines_();
    }
    std::cerr << "volcart::texturing::abf: Iterations: " << i
              << " || Final norm: " << norm << " || Limit: " << limit_
              << std::endl;
}

///// Helpers - ABF /////
// Compute the sine and cosine for each angle in the HEM
void AngleBasedFlattening::compute_sines_()
{
    for (auto e = heMesh_.getEdgesBegin(); e != heMesh_.getEdgesEnd(); ++e) {
        (*e)->angle->sine = sin((*e)->angle->alpha);
        (*e)->angle->cosine = cos((*e)->angle->alpha);
    }
}

// Calculate the magnitude of the error vector
double AngleBasedFlattening::compute_gradient_()
{
    double norm = 0.0;

    // Gradient alpha per face
    for (auto f = heMesh_.getFace(0); f; f = f->nextLink) {

        auto e0 = f->edge, e1 = e0->next, e2 = e1->next;
        double gTriangle, gAlpha0, gAlpha1, gAlpha2;

        gAlpha0 = compute_gradient_alpha_(e0);
        gAlpha1 = compute_gradient_alpha_(e1);
        gAlpha2 = compute_gradient_alpha_(e2);

        e0->angle->bAlpha = -gAlpha0;
        e1->angle->bAlpha = -gAlpha1;
        e2->angle->bAlpha = -gAlpha2;

        norm += gAlpha0 * gAlpha0 + gAlpha1 * gAlpha1 + gAlpha2 * gAlpha2;
        gTriangle =
            e0->angle->alpha + e1->angle->alpha + e2->angle->alpha - M_PI;
        f->bTriangle = -gTriangle;
        norm += gTriangle * gTriangle;
    }

    // Planarity check for interior verts
    for (auto v = heMesh_.getVert(0); v; v = v->nextLink) {
        // Only consider interior points
        if (!v->interior()) {
            continue;
        }

        auto iId = interior_[v->id];
        double gplanar = -2 * M_PI;
        double glength;

        volcart::HalfEdgeMesh::EdgePtr e = v->edge;
        do {
            gplanar += e->angle->alpha;
            e = e->next->next->pair;
        } while (e && (e != v->edge));

        bInterior_[iId] = -gplanar;
        norm += gplanar * gplanar;

        glength = compute_sin_product_(v);
        bInterior_[heMesh_.getNumberOfInteriorPoints() + iId] = -glength;
        norm += glength * glength;
    }

    return norm;
}

// Edge length constraint calculation
double AngleBasedFlattening::compute_sin_product_(
    const volcart::HalfEdgeMesh::VertPtr& v)
{
    volcart::HalfEdgeMesh::EdgePtr e0, e1, e2;
    double sin1, sin2;
    sin1 = sin2 = 1.0;

    e0 = v->edge;
    do {
        e1 = e0->next;
        e2 = e0->next->next;

        sin1 *= e1->angle->sine;
        sin2 *= e2->angle->sine;

        e0 = e0->next->next->pair;
    } while (e0 && (e0 != v->edge));

    return (sin1 - sin2);
}

// Same with alternate vertex aId
double AngleBasedFlattening::compute_sin_product_(
    const volcart::HalfEdgeMesh::VertPtr& v, volcart::HalfEdgeMesh::IDType aId)
{
    volcart::HalfEdgeMesh::EdgePtr e0, e1, e2;
    double sin1, sin2;
    sin1 = sin2 = 1.0;

    e0 = v->edge;
    do {
        e1 = e0->next;
        e2 = e0->next->next;

        if (aId == e1->id) {
            /* we are computing a derivative for this angle,
             * so we use cos and drop the other part */
            sin1 *= e1->angle->cosine;
            sin2 = 0.0;
        } else {
            sin1 *= e1->angle->sine;
        }

        if (aId == e2->id) {
            /* see above */
            sin1 = 0.0;
            sin2 *= e2->angle->cosine;
        } else {
            sin2 *= e2->angle->sine;
        }

        e0 = e0->next->next->pair;
    } while (e0 && (e0 != v->edge));

    return (sin1 - sin2);
}

// Fill the matrix and attempt to solve
bool AngleBasedFlattening::invert_matrix_()
{
    // Create a new solver + context
    bool success;
    LinearSolver* context;
    volcart::HalfEdgeMesh::IDType ninterior = interior_.size();

    context = EIG_linear_solver_new(0, ninterior * 2, 1);

    // Add the bInterior_ points to RHS
    for (volcart::HalfEdgeMesh::IDType i = 0; i < bInterior_.size(); ++i) {
        EIG_linear_solver_right_hand_side_add(context, 0, i, bInterior_[i]);
    }

    // For each face
    int counter = 0;
    for (auto f = heMesh_.getFace(0); f; f = f->nextLink, ++counter) {
        // Setup a matrix
        double wi1, wi2, wi3, b, si, beta[3], j2[3][3], w[3][3];
        double row1[6], row2[6], row3[6];
        int vid[6];

        volcart::HalfEdgeMesh::EdgePtr e0 = f->edge, e1 = e0->next,
                                       e2 = e1->next;
        volcart::HalfEdgeMesh::VertPtr v0 = e0->vert, v1 = e1->vert,
                                       v2 = e2->vert;

        wi1 = 1.0f / e0->angle->weight;
        wi2 = 1.0f / e1->angle->weight;
        wi3 = 1.0f / e2->angle->weight;

        /* bstar1 = (J1*dInv*bAlpha - bTriangle) */
        b = e0->angle->bAlpha * wi1;
        b += e1->angle->bAlpha * wi2;
        b += e2->angle->bAlpha * wi3;
        b -= f->bTriangle;

        /* si = J1*d*J1t */
        si = 1.0f / (wi1 + wi2 + wi3);

        /* J1t*si*bstar1 - bAlpha */
        beta[0] = b * si - e0->angle->bAlpha;
        beta[1] = b * si - e1->angle->bAlpha;
        beta[2] = b * si - e2->angle->bAlpha;

        /* use this later for computing other lambda's */
        f->bstar = b;
        f->dstar = si;

        /* set matrix */
        w[0][0] = si - e0->angle->weight;
        w[0][1] = si;
        w[0][2] = si;
        w[1][0] = si;
        w[1][1] = si - e1->angle->weight;
        w[1][2] = si;
        w[2][0] = si;
        w[2][1] = si;
        w[2][2] = si - e2->angle->weight;

        vid[0] = vid[1] = vid[2] = vid[3] = vid[4] = vid[5] = -1;

        // Add each vert to RHS if interior
        if (v0->interior()) {
            uint64_t iId = vid[0] = interior_[v0->id];
            vid[3] = ninterior + iId;

            j2dt_.at<double>(e0->id, 0) = j2[0][0] = 1.0f * wi1;
            j2dt_.at<double>(e1->id, 0) = j2[1][0] =
                compute_sin_product_(e0->vert, e1->id) * wi2;
            j2dt_.at<double>(e2->id, 0) = j2[2][0] =
                compute_sin_product_(e0->vert, e2->id) * wi3;

            EIG_linear_solver_right_hand_side_add(
                context, 0, iId, j2[0][0] * beta[0]);
            EIG_linear_solver_right_hand_side_add(
                context, 0, ninterior + iId,
                j2[1][0] * beta[1] + j2[2][0] * beta[2]);

            row1[0] = j2[0][0] * w[0][0];
            row2[0] = j2[0][0] * w[1][0];
            row3[0] = j2[0][0] * w[2][0];

            row1[3] = j2[1][0] * w[0][1] + j2[2][0] * w[0][2];
            row2[3] = j2[1][0] * w[1][1] + j2[2][0] * w[1][2];
            row3[3] = j2[1][0] * w[2][1] + j2[2][0] * w[2][2];
        }

        if (v1->interior()) {
            uint64_t iId = vid[1] = interior_[v1->id];
            vid[4] = ninterior + iId;

            j2dt_.at<double>(e0->id, 1) = j2[0][1] =
                compute_sin_product_(e1->vert, e0->id) * wi1;
            j2dt_.at<double>(e1->id, 1) = j2[1][1] = 1.0f * wi2;
            j2dt_.at<double>(e2->id, 1) = j2[2][1] =
                compute_sin_product_(e1->vert, e2->id) * wi3;

            EIG_linear_solver_right_hand_side_add(
                context, 0, iId, j2[1][1] * beta[1]);
            EIG_linear_solver_right_hand_side_add(
                context, 0, ninterior + iId,
                j2[0][1] * beta[0] + j2[2][1] * beta[2]);

            row1[1] = j2[1][1] * w[0][1];
            row2[1] = j2[1][1] * w[1][1];
            row3[1] = j2[1][1] * w[2][1];

            row1[4] = j2[0][1] * w[0][0] + j2[2][1] * w[0][2];
            row2[4] = j2[0][1] * w[1][0] + j2[2][1] * w[1][2];
            row3[4] = j2[0][1] * w[2][0] + j2[2][1] * w[2][2];
        }

        if (v2->interior()) {
            uint64_t iId = vid[2] = interior_[v2->id];
            vid[5] = ninterior + iId;

            j2dt_.at<double>(e0->id, 2) = j2[0][2] =
                compute_sin_product_(e2->vert, e0->id) * wi1;
            j2dt_.at<double>(e1->id, 2) = j2[1][2] =
                compute_sin_product_(e2->vert, e1->id) * wi2;
            j2dt_.at<double>(e2->id, 2) = j2[2][2] = 1.0f * wi3;

            EIG_linear_solver_right_hand_side_add(
                context, 0, iId, j2[2][2] * beta[2]);
            EIG_linear_solver_right_hand_side_add(
                context, 0, ninterior + iId,
                j2[0][2] * beta[0] + j2[1][2] * beta[1]);

            row1[2] = j2[2][2] * w[0][2];
            row2[2] = j2[2][2] * w[1][2];
            row3[2] = j2[2][2] * w[2][2];

            row1[5] = j2[0][2] * w[0][0] + j2[1][2] * w[0][1];
            row2[5] = j2[0][2] * w[1][0] + j2[1][2] * w[1][1];
            row3[5] = j2[0][2] * w[2][0] + j2[1][2] * w[2][1];
        }

        for (int i = 0; i < 3; ++i) {
            int r = vid[i];

            // Unset condition
            if (r == -1) {
                continue;
            }

            for (int j = 0; j < 6; ++j) {
                int c = vid[j];

                if (c == -1) {
                    continue;
                }

                if (i == 0) {
                    EIG_linear_solver_matrix_add(
                        context, r, c, j2[0][i] * row1[j]);
                } else {
                    EIG_linear_solver_matrix_add(
                        context, r + ninterior, c, j2[0][i] * row1[j]);
                }

                if (i == 1) {
                    EIG_linear_solver_matrix_add(
                        context, r, c, j2[1][i] * row2[j]);
                } else {
                    EIG_linear_solver_matrix_add(
                        context, r + ninterior, c, j2[1][i] * row2[j]);
                }

                if (i == 2) {
                    EIG_linear_solver_matrix_add(
                        context, r, c, j2[2][i] * row3[j]);
                } else {
                    EIG_linear_solver_matrix_add(
                        context, r + ninterior, c, j2[2][i] * row3[j]);
                }
            }
        }
    }

    // solve
    success = EIG_linear_solver_solve(context);

    // if successful, update the HEM
    if (success) {
        for (auto f = heMesh_.getFace(0); f; f = f->nextLink) {
            double dlambda1, pre[3], dalpha;

            volcart::HalfEdgeMesh::EdgePtr e0 = f->edge, e1 = e0->next,
                                           e2 = e1->next;
            volcart::HalfEdgeMesh::VertPtr v0 = e0->vert, v1 = e1->vert,
                                           v2 = e2->vert;

            pre[0] = pre[1] = pre[2] = 0.0;

            if (v0->interior()) {
                auto iId = interior_[v0->id];
                double x = EIG_linear_solver_variable_get(context, 0, iId);
                double x2 =
                    EIG_linear_solver_variable_get(context, 0, ninterior + iId);
                pre[0] += j2dt_.at<double>(e0->id, 0) * x;
                pre[1] += j2dt_.at<double>(e1->id, 0) * x2;
                pre[2] += j2dt_.at<double>(e2->id, 0) * x2;
            }

            if (v1->interior()) {
                auto iId = interior_[v1->id];
                double x = EIG_linear_solver_variable_get(context, 0, iId);
                double x2 =
                    EIG_linear_solver_variable_get(context, 0, ninterior + iId);
                pre[0] += j2dt_.at<double>(e0->id, 1) * x2;
                pre[1] += j2dt_.at<double>(e1->id, 1) * x;
                pre[2] += j2dt_.at<double>(e2->id, 1) * x2;
            }

            if (v2->interior()) {
                auto iId = interior_[v2->id];
                double x = EIG_linear_solver_variable_get(context, 0, iId);
                double x2 =
                    EIG_linear_solver_variable_get(context, 0, ninterior + iId);
                pre[0] += j2dt_.at<double>(e0->id, 2) * x2;
                pre[1] += j2dt_.at<double>(e1->id, 2) * x2;
                pre[2] += j2dt_.at<double>(e2->id, 2) * x;
            }

            dlambda1 = pre[0] + pre[1] + pre[2];
            dlambda1 = f->dstar * (f->bstar - dlambda1);

            f->lambdaTriangle += dlambda1;

            dalpha = (e0->angle->bAlpha - dlambda1);
            e0->angle->alpha += dalpha / e0->angle->weight - pre[0];

            dalpha = (e1->angle->bAlpha - dlambda1);
            e1->angle->alpha += dalpha / e1->angle->weight - pre[1];

            dalpha = (e2->angle->bAlpha - dlambda1);
            e2->angle->alpha += dalpha / e2->angle->weight - pre[2];

            /* clamp all the angles to between 0 & pi*/
            auto e = f->edge;
            do {
                if (e->angle->alpha > M_PI) {
                    e->angle->alpha = M_PI;
                } else if (e->angle->alpha < 0.0f) {
                    e->angle->alpha = 0.0f;
                }
                e = e->next;
            } while (e != f->edge);
        }

        volcart::HalfEdgeMesh::IDType pId, iId;
        for (auto it : interior_) {
            std::tie(pId, iId) = it;
            heMesh_.getVert(pId)->lambdaPlanar +=
                EIG_linear_solver_variable_get(context, 0, iId);
            heMesh_.getVert(pId)->lambdaLength +=
                EIG_linear_solver_variable_get(context, 0, ninterior + iId);
        }
    }

    // delete context
    EIG_linear_solver_delete(context);

    // return success state
    return success;
}

///// LSCM Loop /////
void AngleBasedFlattening::solve_lscm_()
{
    // find two pins and compute their positions
    std::tie(pin0_, pin1_) = get_min_z_point_ids_();
    compute_pin_uv_();

    // Setup solver context
    LinearSolver* context;
    context = EIG_linear_least_squares_solver_new(
        2 * heMesh_.getNumberOfFaces(), 2 * heMesh_.getNumberOfVerts(), 1);

    // Add pins to solver
    EIG_linear_solver_variable_lock(context, 2 * pin0_);
    EIG_linear_solver_variable_lock(context, 2 * pin0_ + 1);
    EIG_linear_solver_variable_lock(context, 2 * pin1_);
    EIG_linear_solver_variable_lock(context, 2 * pin1_ + 1);

    EIG_linear_solver_variable_set(
        context, 0, 2 * pin0_, heMesh_.getVert(pin0_)->uv[0]);
    EIG_linear_solver_variable_set(
        context, 0, 2 * pin0_ + 1, heMesh_.getVert(pin0_)->uv[1]);
    EIG_linear_solver_variable_set(
        context, 0, 2 * pin1_, heMesh_.getVert(pin1_)->uv[0]);
    EIG_linear_solver_variable_set(
        context, 0, 2 * pin1_ + 1, heMesh_.getVert(pin1_)->uv[1]);

    // Construct matrix
    volcart::HalfEdgeMesh::IDType row = 0;
    for (auto f = heMesh_.getFacesBegin(); f != heMesh_.getFacesEnd(); ++f) {
        volcart::HalfEdgeMesh::EdgePtr e0 = (*f)->edge, e1 = e0->next,
                                       e2 = e1->next;

        volcart::HalfEdgeMesh::IDType v0 = e0->vert->id;
        volcart::HalfEdgeMesh::IDType v1 = e1->vert->id;
        volcart::HalfEdgeMesh::IDType v2 = e2->vert->id;

        double a0 = e0->angle->alpha;
        double a1 = e1->angle->alpha;
        double a2 = e2->angle->alpha;

        // Find max sin from angles
        double sin0 = sin(a0);
        double sin1 = sin(a1);
        double sin2 = sin(a2);

        double sinmax = std::max(sin0, std::max(sin1, sin2));

        // Shift verts for stable order
        // Careful. Only use these values going forward through the loop
        if (sin2 != sinmax) {
            SHIFT3(volcart::HalfEdgeMesh::IDType, v0, v1, v2);
            SHIFT3(double, a0, a1, a2);
            SHIFT3(double, sin0, sin1, sin2);

            if (sin1 == sinmax) {
                SHIFT3(QuadPointIdentifier, v0, v1, v2);
                SHIFT3(double, a0, a1, a2);
                SHIFT3(double, sin0, sin1, sin2);
            }
        }

        // Setup angle based lscm
        double ratio = (sin2 == 0.0) ? 1.0 : sin1 / sin2;
        double cosine = cos(a0) * ratio;
        double sine = sin0 * ratio;

        EIG_linear_solver_matrix_add(context, row, 2 * v0, cosine - 1.0f);
        EIG_linear_solver_matrix_add(context, row, 2 * v0 + 1, -sine);
        EIG_linear_solver_matrix_add(context, row, 2 * v1, -cosine);
        EIG_linear_solver_matrix_add(context, row, 2 * v1 + 1, sine);
        EIG_linear_solver_matrix_add(context, row, 2 * v2, 1.0);
        row++;

        EIG_linear_solver_matrix_add(context, row, 2 * v0, sine);
        EIG_linear_solver_matrix_add(context, row, 2 * v0 + 1, cosine - 1.0f);
        EIG_linear_solver_matrix_add(context, row, 2 * v1, -sine);
        EIG_linear_solver_matrix_add(context, row, 2 * v1 + 1, -cosine);
        EIG_linear_solver_matrix_add(context, row, 2 * v2 + 1, 1.0);
        row++;
    }

    // Solve
    bool success = EIG_linear_solver_solve(context);
    if (!success) {
        throw std::runtime_error("Failed to solve lscm.");
    }

    // Update UVs
    for (auto v = heMesh_.getVertsBegin(); v != heMesh_.getVertsEnd(); ++v) {
        (*v)->uv[0] = EIG_linear_solver_variable_get(context, 0, 2 * (*v)->id);
        (*v)->uv[1] =
            EIG_linear_solver_variable_get(context, 0, 2 * (*v)->id + 1);
    }

    // Cleanup context
    EIG_linear_solver_delete(context);
}

///// Helpers - LSCM /////
// Get the id's of the points with the minimum and maximum 3D positions
std::pair<volcart::HalfEdgeMesh::IDType, volcart::HalfEdgeMesh::IDType>
AngleBasedFlattening::get_min_max_point_ids_()
{
    cv::Vec3d min(1e20), max(-1e20);
    volcart::HalfEdgeMesh::IDType minVert = 0;
    volcart::HalfEdgeMesh::IDType maxVert = 0;

    for (auto it = heMesh_.getVertsBegin(); it != heMesh_.getVertsEnd(); ++it) {

        // Min
        if ((*it)->xyz[0] < min[0] && (*it)->xyz[1] < min[1] &&
            (*it)->xyz[2] < min[2]) {
            min = (*it)->xyz;
            minVert = (*it)->id;
            // Max
        } else if (
            (*it)->xyz[0] > max[0] && (*it)->xyz[0] > max[1] &&
            (*it)->xyz[0] > max[2]) {
            max = (*it)->xyz;
            maxVert = (*it)->id;
        }
    }

    return std::make_pair(minVert, maxVert);
}

// Get the ID's of two points near the minimum z position
std::pair<volcart::HalfEdgeMesh::IDType, volcart::HalfEdgeMesh::IDType>
AngleBasedFlattening::get_min_z_point_ids_()
{
    auto it = heMesh_.getBoundaryBegin();
    double min = (*it)->xyz[2];
    volcart::HalfEdgeMesh::IDType minVert = (*it)->id;
    volcart::HalfEdgeMesh::IDType nextVert = (*(it + 1))->id;

    for (; it != heMesh_.getBoundaryEnd(); ++it) {

        // Min
        if ((*it)->xyz[2] < min) {
            min = (*it)->xyz[2];
            minVert = (*it)->id;
            if ((it + 1) != heMesh_.getBoundaryEnd()) {
                nextVert = (*(it + 1))->id;
            } else {
                nextVert = (*(it - 1))->id;
            }
        }
    }

    return std::make_pair(minVert, nextVert);
}

// Generate a good starting UV position for the two starting "pinned" points
// This is supposedly arbitrary
void AngleBasedFlattening::compute_pin_uv_()
{
    auto p0xyz = heMesh_.getVert(pin0_)->xyz;
    auto p1xyz = heMesh_.getVert(pin1_)->xyz;
    cv::Vec2d xy1(p0xyz[0], p0xyz[1]);
    cv::Vec2d xy2(p1xyz[0], p1xyz[1]);
    double uDist = cv::norm(xy1, xy2);
    double vDist = p0xyz[2] - p1xyz[2];
    heMesh_.getVert(pin0_)->uv = cv::Vec2d(0.0, 0.0);
    heMesh_.getVert(pin1_)->uv = cv::Vec2d(uDist, vDist);
}
