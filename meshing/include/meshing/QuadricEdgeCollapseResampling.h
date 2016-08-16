//
// Created by Hannah Hatch on 8/12/16.
//

#ifndef VC_QUADRICEDGECOLLAPSERESAMPLING_H
#define VC_QUADRICEDGECOLLAPSERESAMPLING_H

#include <iostream>
#include "common/vc_defines.h"
#include "vcg/vcg/complex/algorithms/local_optimization/tri_edge_collapse_quadric.h"
#include "vcg/complex/complex.h"
#include "vcg/complex/allocate.h"
#include "vcg/complex/algorithms/local_optimization.h"
#include "vcg/complex/algorithms/edge_collapse.h"
#include "vcg/math/quadric.h"


namespace volcart {
    namespace meshing{
        class QuadricEdgeCollapseResampling {
            class MyVertex; class MyEdge; class MyFace;
            struct MyUsedTypes: public vcg::UsedTypes<vcg::Use<MyVertex>    ::AsVertexType,
                    vcg::Use<MyEdge>      ::AsEdgeType,
                    vcg::Use<MyFace>      ::AsFaceType>{};

            class MyVertex : public vcg::Vertex< MyUsedTypes, vcg::vertex::VFAdj, vcg::vertex::Coord3f, vcg::vertex::Normal3f, vcg::vertex::Mark, vcg::vertex::Qualityf, vcg::vertex::BitFlags>{
            public:
                vcg::math::Quadric<double>&Qd(){return q;}
            private:
                vcg::math::Quadric<double> q;
            };
            class MyFace : public vcg::Face< MyUsedTypes, vcg::face::VFAdj, vcg::face::VertexRef, vcg::face::BitFlags>{};
            class MyEdge : public vcg::Edge <MyUsedTypes > {};
            class MyMesh : public vcg::tri::TriMesh<std::vector<MyVertex>, std::vector<MyFace>> {};

            MyMesh::PerVertexAttributeHandle<unsigned long> vertex_id;
            typedef vcg::tri::BasicVertexPair<MyVertex> VertexPair;

            class MyTriEdgeCollapse: public vcg::tri::TriEdgeCollapseQuadric<MyMesh, VertexPair, MyTriEdgeCollapse, vcg::tri::QInfoStandard<MyVertex> > {
            public:
                typedef vcg::tri::TriEdgeCollapseQuadric<MyMesh,VertexPair,MyTriEdgeCollapse,vcg::tri::QInfoStandard<MyVertex> > TECQ;
                typedef MyMesh::VertexType::EdgeType EdgeType;
                inline MyTriEdgeCollapse(const VertexPair &p, int i, vcg::BaseParameterClass *pp) : TECQ(p,i,pp){}
            };
        private:
            MyMesh input;
        public:
            //Initializers//
            QuadricEdgeCollapseResampling();
            QuadricEdgeCollapseResampling(VC_MeshType::Pointer mesh);

            //Set All Parameters//
            void setMesh(VC_MeshType::Pointer mesh);
            void setDefaultParams();

            void setBoundaryWeight(double weight) {_collapseParams.CosineThr = weight;}
            void setCosineThr(double thr) {_collapseParams.CosineThr = thr;  }
            void setFastPreserveBoundary(bool set) {_collapseParams.FastPreserveBoundary = set;}
            void setNormalCheck(bool set) {_collapseParams.NormalCheck = set;  }
            void setNormalThrRad(double rad) {_collapseParams.NormalThrRad = rad; }
            void setOptimalPlacement(bool set) {_collapseParams.OptimalPlacement = set;}
            void setPreserveTopology(bool set) {_collapseParams.PreserveTopology = set; }
            void setPreserveBoundary(bool set) {_collapseParams.PreserveBoundary = set;}
            void setQuadricEpsilon(double epsilon) {_collapseParams.QuadricEpsilon = epsilon;}
            void setQualityCheck(bool set) {_collapseParams.QualityCheck = set;}
            void setQualityQuadric(bool set) {_collapseParams.QualityQuadric = set;}
            void setQualityThr(double thr) {_collapseParams.QualityThr = thr;}
            void setQualityWeight(bool set) {_collapseParams.QualityWeight = set;}
            void setQualityWeightFactor(double factor) {_collapseParams.QualityWeight = factor;}
            void setScaleFactor(double scale) {_collapseParams.ScaleFactor = scale;}
            void setScaleIndependent(bool set) {_collapseParams.ScaleIndependent = set;}
            void setUseArea(bool set) {_collapseParams.UseArea = set;}
            void setUseVertexWeight(bool set) {_collapseParams.UseVertexWeight = set;}


            VC_MeshType::Pointer getMesh();
            void compute(int iterations);
        private:
            VC_MeshType::Pointer _inputMesh;
            VC_MeshType::Pointer _outputMesh = VC_MeshType::New();
            vcg::tri::TriEdgeCollapseQuadricParameter _collapseParams;

        };

    } //meshing
} //volcart

#endif //VC_QUADRICEDGECOLLAPSERESAMPLING_H
