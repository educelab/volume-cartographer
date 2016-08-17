//
// Created by Hannah Hatch on 8/12/16.
//

#pragma once

#include <iostream>
#include "common/vc_defines.h"
#include "vcg/vcg/complex/algorithms/local_optimization/tri_edge_collapse_quadric.h"
#include "vcg/complex/complex.h"
#include "vcg/complex/allocate.h"
#include "vcg/complex/algorithms/local_optimization.h"
#include "vcg/complex/algorithms/edge_collapse.h"
#include "vcg/math/quadric.h"
#include "vcg/container/simple_temporary_data.h"


namespace volcart {
    namespace meshing{
        class QuadricEdgeCollapseResampling {
            //All of these classes set up the elements necessary to use a vcg mesh
            class MyVertex; class MyEdge; class MyFace;
            struct MyUsedTypes: public vcg::UsedTypes<vcg::Use<MyVertex>::AsVertexType,vcg::Use<MyEdge>::AsEdgeType,vcg::Use<MyFace>::AsFaceType>{};

            class MyVertex : public vcg::Vertex< MyUsedTypes, vcg::vertex::VFAdj, vcg::vertex::Coord3f, vcg::vertex::Normal3f, vcg::vertex::Mark, vcg::vertex::Qualityf, vcg::vertex::BitFlags>{
            public:
                //Used to compute error as a result of an edge collapse
                vcg::math::Quadric<double>&Qd(){return q;}
            private:
                vcg::math::Quadric<double> q;
            };

            class MyFace : public vcg::Face< MyUsedTypes, vcg::face::VFAdj, vcg::face::VertexRef, vcg::face::BitFlags>{};
            class MyEdge : public vcg::Edge <MyUsedTypes > {};
            class MyMesh : public vcg::tri::TriMesh<std::vector<MyVertex>, std::vector<MyFace>> {};

            //Used for decimation
            typedef vcg::tri::BasicVertexPair<MyVertex> VertexPair;

            class MyTriEdgeCollapse: public vcg::tri::TriEdgeCollapseQuadric<MyMesh, VertexPair, MyTriEdgeCollapse, vcg::tri::QInfoStandard<MyVertex> > {
            public:
                typedef vcg::tri::TriEdgeCollapseQuadric<MyMesh,VertexPair,MyTriEdgeCollapse,vcg::tri::QInfoStandard<MyVertex> > TECQ;
                typedef MyMesh::VertexType::EdgeType EdgeType;
                inline MyTriEdgeCollapse(const VertexPair &p, int i, vcg::BaseParameterClass *pp) : TECQ(p,i,pp){}
            };
        private:
            MyMesh _vcgInput;
        public:
            //Initializers//
            QuadricEdgeCollapseResampling();
            QuadricEdgeCollapseResampling(VC_MeshType::Pointer mesh);

            //Set All Parameters//
            void setMesh(VC_MeshType::Pointer mesh);
            void setDefaultParams();

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

            //Note to Seth: Not sure if that's the best way to say what defaults are, or if I should do a big list

            VC_MeshType::Pointer getMesh();
            void compute(int iterations);
        private:
            void _convertMeshtoVCG();
            VC_MeshType::Pointer _itkInput;
            VC_MeshType::Pointer _outputMesh;
            vcg::tri::TriEdgeCollapseQuadricParameter _collapseParams;

        };

    } //meshing
} //volcart

