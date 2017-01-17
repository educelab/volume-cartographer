// Angle-based Flattening (abf)
// Created by Seth Parker on 6/9/16.
// Angle-based Flattening implementation ported from the same in Blender
// Note: This is borrowed very heavily from Blender's implementation.

// This class attempts to find the ideal angles that minimize the angular
// distortion of the parameterized mesh.
// These idealized angles are then fed into a least-squares conformal maps
// algorithm which solves for the actual
// parameterized UV positions.
#pragma once

#include <exception>
#include <iostream>
#include <memory>

#include <itkQuadEdgeMeshBoundaryEdgesMeshFunction.h>
#include <opencv2/core.hpp>

#include "core/types/HalfEdgeMesh.h"
#include "core/types/UVMap.h"
#include "core/vc_defines.h"

/*
 * Switch to this when we validate that it does the same as the macro
template <typename T>
void Shift3(T& a, T& b, T& c)
{
    T tmp = a;
    a = c;
    c = b;
    b = tmp;
}
*/
// This is terrible but it'll work for now - SP
#define SHIFT3(type, a, b, c) \
    {                         \
        type tmp;             \
        tmp = a;              \
        (a) = c;              \
        (c) = b;              \
        (b) = tmp;            \
    }

namespace volcart
{
namespace texturing
{

class AngleBasedFlattening
{
public:
    ///// Constructors/Destructors /////
    AngleBasedFlattening()
        : useABF_{true}, maxABFIterations_{DEFAULT_MAX_ABF_ITERATIONS}
    {
    }

    explicit AngleBasedFlattening(ITKMesh::Pointer mesh)
        : useABF_{true}
        , maxABFIterations_{DEFAULT_MAX_ABF_ITERATIONS}
        , mesh_{mesh}
    {
    }

    ///// Access Functions /////
    // Set inputs
    void setMesh(const ITKMesh::Pointer& mesh) { mesh_ = mesh; }

    // Get outputs
    ITKMesh::Pointer getMesh();
    volcart::UVMap getUVMap();

    ///// Parameters /////
    void setUseABF(bool a) { useABF_ = a; }
    void setABFMaxIterations(int i) { maxABFIterations_ = i; }

    ///// Process /////
    void compute();

    ///// Default values /////
    static const int DEFAULT_MAX_ABF_ITERATIONS = 5;

private:
    ///// Setup /////
    void fill_half_edge_mesh_();

    ///// Solve - ABF /////
    void solve_abf_();
    void scale_();

    void compute_sines_();
    double compute_gradient_();
    double compute_gradient_alpha_(const HalfEdgeMesh::EdgePtr& e0)
    {
        return std::pow(e0->angle->alpha - e0->angle->phi, 2) *
               e0->angle->weight;
    }
    double compute_sin_product_(const HalfEdgeMesh::VertPtr& v);
    double compute_sin_product_(
        const HalfEdgeMesh::VertPtr& v, HalfEdgeMesh::IDType aId);
    bool invert_matrix_();

    // Parameters
    bool useABF_;  // If false, only compute LSCM parameterization [default:
                   // true]
    int maxABFIterations_;  // Max number of iterations
    double limit_;          // Minimization limit

    ///// LSCM Loop /////
    void solve_lscm_();

    ///// Helper Functions - LSCM /////
    std::pair<HalfEdgeMesh::IDType, HalfEdgeMesh::IDType>
    get_min_max_point_ids_();
    std::pair<HalfEdgeMesh::IDType, HalfEdgeMesh::IDType>
    get_min_z_point_ids_();
    void compute_pin_uv_();

    ///// Storage /////
    ITKMesh::Pointer mesh_;  // input mesh
    HalfEdgeMesh heMesh_;    // half-edge mesh for processing

    // Interior Vertices
    // < id in quadMesh, id in list >
    std::map<volcart::QuadPointIdentifier, volcart::QuadPointIdentifier>
        interior_;

    std::vector<double> bInterior_;
    cv::Mat j2dt_;

    // Pinned Point IDs
    HalfEdgeMesh::IDType pin0_;
    HalfEdgeMesh::IDType pin1_;
};
}
}
