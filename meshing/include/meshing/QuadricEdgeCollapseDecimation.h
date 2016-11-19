//
// Created by Hannah Hatch on 8/12/16.
//
#pragma once

#include <iostream>
// vcglib has order-specific includes, so disable clang-format for these so they
// don't get reordered
// clang-format off
#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/local_optimization/tri_edge_collapse_quadric.h>
// clang-format on
#include "core/vc_defines.h"

namespace volcart
{
namespace meshing
{

class QuadricEdgeCollapseDecimation
{

    ///// Basic Datastructure (VCG Mesh) /////
    class VcgVertex;
    class VcgEdge;
    class VcgFace;

    struct VcgUsedTypes : public vcg::UsedTypes<
                              vcg::Use<VcgVertex>::AsVertexType,
                              vcg::Use<VcgEdge>::AsEdgeType,
                              vcg::Use<VcgFace>::AsFaceType> {
    };

    class VcgVertex : public vcg::Vertex<
                          VcgUsedTypes,
                          vcg::vertex::VFAdj,
                          vcg::vertex::Coord3d,
                          vcg::vertex::Normal3d,
                          vcg::vertex::Mark,
                          vcg::vertex::Qualityd,
                          vcg::vertex::BitFlags>
    {
    public:
        // Used to compute error as a result of an edge collapse
        vcg::math::Quadric<double>& Qd() { return q; }
    private:
        vcg::math::Quadric<double> q;
    };

    class VcgFace : public vcg::Face<
                        VcgUsedTypes,
                        vcg::face::VFAdj,
                        vcg::face::VertexRef,
                        vcg::face::BitFlags>
    {
    };

    class VcgEdge : public vcg::Edge<VcgUsedTypes>
    {
    };
    class VcgMesh
        : public vcg::tri::TriMesh<std::vector<VcgVertex>, std::vector<VcgFace>>
    {
    };

    ///// Edge Collapse typedefs & classes /////
    typedef vcg::tri::BasicVertexPair<VcgVertex> VertexPair;

    class VcgTriEdgeCollapse : public vcg::tri::TriEdgeCollapseQuadric<
                                   VcgMesh,
                                   VertexPair,
                                   VcgTriEdgeCollapse,
                                   vcg::tri::QInfoStandard<VcgVertex>>
    {
    public:
        typedef vcg::tri::TriEdgeCollapseQuadric<
            VcgMesh,
            VertexPair,
            VcgTriEdgeCollapse,
            vcg::tri::QInfoStandard<VcgVertex>>
            TECQ;
        typedef VcgMesh::VertexType::EdgeType EdgeType;
        inline VcgTriEdgeCollapse(
            const VertexPair& p, int i, vcg::BaseParameterClass* pp)
            : TECQ(p, i, pp){};
    };

public:
    // Initializers
    QuadricEdgeCollapseDecimation();
    QuadricEdgeCollapseDecimation(ITKMesh::Pointer mesh);

    // Set All Parameters
    void setMesh(ITKMesh::Pointer mesh);
    void setDefaultParams();
    void setAllParams(vcg::tri::TriEdgeCollapseQuadricParameter newParams)
    {
        collapseParams_ = newParams;
    }

    // Defaults set by VCG, with the exception of PreserveBoundary and
    // PreserveTopology
    void setDesiredFaces(size_t n) { desiredFaces_ = n; };
    void setBoundaryWeight(double weight)
    {
        collapseParams_.CosineThr = weight;
    }  // Default: .5
    void setCosineThr(double thr)
    {
        collapseParams_.CosineThr = thr;
    }  // Default: cos(M_PI/2)
    void setFastPreserveBoundary(bool set)
    {
        collapseParams_.FastPreserveBoundary = set;
    }  // Default: false
    void setNormalCheck(bool set)
    {
        collapseParams_.NormalCheck = set;
    }  // Default: false
    void setNormalThrRad(double rad)
    {
        collapseParams_.NormalThrRad = rad;
    }  // Default: M_PI/2
    void setOptimalPlacement(bool set)
    {
        collapseParams_.OptimalPlacement = set;
    }  // Default: true
    void setPreserveTopology(bool set)
    {
        collapseParams_.PreserveTopology = set;
    }  // Default: true
    void setPreserveBoundary(bool set)
    {
        collapseParams_.PreserveBoundary = set;
    }  // Default: true
    void setQuadricEpsilon(double epsilon)
    {
        collapseParams_.QuadricEpsilon = epsilon;
    }  // Default:1e-15
    void setQualityCheck(bool set)
    {
        collapseParams_.QualityCheck = set;
    }  // Default: true
    void setQualityQuadric(bool set)
    {
        collapseParams_.QualityQuadric = set;
    }  // Default: false
    void setQualityThr(double thr)
    {
        collapseParams_.QualityThr = thr;
    }  // Default: .3
    void setQualityWeight(bool set)
    {
        collapseParams_.QualityWeight = set;
    }  // Default: false
    void setQualityWeightFactor(double factor)
    {
        collapseParams_.QualityWeight = factor;
    }  // Default: 100.0
    void setScaleFactor(double scale)
    {
        collapseParams_.ScaleFactor = scale;
    }  // Default: 1.0
    void setScaleIndependent(bool set)
    {
        collapseParams_.ScaleIndependent = set;
    }  // Default: true
    void setUseArea(bool set)
    {
        collapseParams_.UseArea = set;
    }  // Default: true;
    void setUseVertexWeight(bool set)
    {
        collapseParams_.UseVertexWeight = set;
    }  // Default: false

    // Processing
    void compute();
    void compute(size_t desiredFaces);

    // Output
    ITKMesh::Pointer getMesh();

private:
    void convertMeshtoVCG_();
    ITKMesh::Pointer itkInput_;
    VcgMesh vcgInput_;
    ITKMesh::Pointer outputMesh_;
    size_t desiredFaces_;
    vcg::tri::TriEdgeCollapseQuadricParameter collapseParams_;

};  // QuadricEdgeCollapse

}  // meshing
}  // volcart
