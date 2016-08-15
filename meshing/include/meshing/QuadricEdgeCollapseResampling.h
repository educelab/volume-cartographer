//
// Created by Hannah Hatch on 8/12/16.
//

#ifndef VC_QUADRICEDGECOLLAPSERESAMPLING_H
#define VC_QUADRICEDGECOLLAPSERESAMPLING_H

#include <iostream>
#include "common/vc_defines.h"
#include "vcg/vcg/complex/algorithms/local_optimization/tri_edge_collapse_quadric.h"
#include "vcg/complex/complex.h:
#include "vcg/complex/algorithms/local_optimization.h"

namespace volcart {
    namespace meshing{
        class QuadricEdgeCollapseResampling {
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
            void compute();
        private:
            VC_MeshType::Pointer _inputMesh;
            vcg::tri::TriEdgeCollapseQuadricParameter _collapseParams;
        };
    } //meshing
} //volcart

#endif //VC_QUADRICEDGECOLLAPSERESAMPLING_H
