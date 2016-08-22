//
// Created by Hannah Hatch on 8/12/16.
//
#pragma once

#include <iostream>
#include "common/vc_defines.h"
#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/local_optimization/tri_edge_collapse_quadric.h>

namespace volcart {
    namespace meshing{
        class QuadricEdgeCollapseResampling {

            ///// Basic Datastructure (VCG Mesh) /////
            class VcgVertex;
            class VcgEdge;
            class VcgFace;

            struct VcgUsedTypes : public vcg::UsedTypes<
                vcg::Use<VcgVertex>::AsVertexType,
                vcg::Use<VcgEdge>::AsEdgeType,
                vcg::Use<VcgFace>::AsFaceType >
            {};

            class VcgVertex : public vcg::Vertex<
                VcgUsedTypes,
                vcg::vertex::VFAdj,
                vcg::vertex::Coord3d,
                vcg::vertex::Normal3d,
                vcg::vertex::Mark,
                vcg::vertex::Qualityd,
                vcg::vertex::BitFlags >
            {
            public:
                //Used to compute error as a result of an edge collapse
                vcg::math::Quadric<double>&Qd(){return q;}
            private:
                vcg::math::Quadric<double> q;
            };

            class VcgFace : public vcg::Face<
                VcgUsedTypes,
                vcg::face::VFAdj,
                vcg::face::VertexRef,
                vcg::face::BitFlags >
            {};

            class VcgEdge : public vcg::Edge<VcgUsedTypes> {};
            class VcgMesh : public vcg::tri::TriMesh<std::vector<VcgVertex>, std::vector<VcgFace>> {};

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
                        vcg::tri::QInfoStandard<VcgVertex> > TECQ;
                typedef VcgMesh::VertexType::EdgeType EdgeType;
                inline VcgTriEdgeCollapse(const VertexPair &p, int i, vcg::BaseParameterClass *pp) : TECQ(p,i,pp){};
            };
        private:
            VcgMesh _vcgInput;
        public:
            // Initializers
            QuadricEdgeCollapseResampling();
            QuadricEdgeCollapseResampling(VC_MeshType::Pointer mesh);

            // Set All Parameters
            void setMesh(VC_MeshType::Pointer mesh);
            void setDefaultParams();
            void setAllParams(vcg::tri::TriEdgeCollapseQuadricParameter newParams) { _collapseParams = newParams; }

            // Defaults set by VCG, with the exception of PreserveBoundary and PreserveTopology
            void setBoundaryWeight(double weight) {_collapseParams.CosineThr = weight;} //Default: .5
            void setCosineThr(double thr) {_collapseParams.CosineThr = thr;  } //Default: cos(M_PI/2)
            void setFastPreserveBoundary(bool set) {_collapseParams.FastPreserveBoundary = set;} //Default: false
            void setNormalCheck(bool set) {_collapseParams.NormalCheck = set;  } //Default: false
            void setNormalThrRad(double rad) {_collapseParams.NormalThrRad = rad; } //Default: M_PI/2
            void setOptimalPlacement(bool set) {_collapseParams.OptimalPlacement = set;} //Default: true
            void setPreserveTopology(bool set) {_collapseParams.PreserveTopology = set; } //Default: true
            void setPreserveBoundary(bool set) {_collapseParams.PreserveBoundary = set;} //Default: true
            void setQuadricEpsilon(double epsilon) {_collapseParams.QuadricEpsilon = epsilon;} //Default:1e-15
            void setQualityCheck(bool set) {_collapseParams.QualityCheck = set;} //Default: true
            void setQualityQuadric(bool set) {_collapseParams.QualityQuadric = set;} //Default: false
            void setQualityThr(double thr) {_collapseParams.QualityThr = thr;} //Default: .3
            void setQualityWeight(bool set) {_collapseParams.QualityWeight = set;} //Default: false
            void setQualityWeightFactor(double factor) {_collapseParams.QualityWeight = factor;} //Default: 100.0
            void setScaleFactor(double scale) {_collapseParams.ScaleFactor = scale;} //Default: 1.0
            void setScaleIndependent(bool set) {_collapseParams.ScaleIndependent = set;} //Default: true
            void setUseArea(bool set) {_collapseParams.UseArea = set;} //Default: true;
            void setUseVertexWeight(bool set) {_collapseParams.UseVertexWeight = set;} //Default: false

            // Processing
            void compute(int iterations);

            // Output
            VC_MeshType::Pointer getMesh();
        private:
            void _convertMeshtoVCG();
            VC_MeshType::Pointer _itkInput;
            VC_MeshType::Pointer _outputMesh;
            vcg::tri::TriEdgeCollapseQuadricParameter _collapseParams;

        };

    } //meshing
} //volcart