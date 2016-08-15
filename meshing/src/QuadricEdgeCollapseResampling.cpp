//
// Created by Hannah Hatch on 8/12/16.
//

#include "meshing/QuadricEdgeCollapseResampling.h"

typedef itk::Vector<double,2>   BasicVertexPair;
typedef vcg::tri::TriEdgeCollapseQuadric<VC_MeshType::Pointer, BasicVertexPair,vcg::tri::QInfoStandard<VC_Vertex>> CollapseType;
namespace volcart{
    namespace meshing{

        QuadricEdgeCollapseResampling::QuadricEdgeCollapseResampling() {
            _inputMesh = nullptr;
            setDefaultParams();
        }

        QuadricEdgeCollapseResampling::QuadricEdgeCollapseResampling(VC_MeshType::Pointer mesh) {
            _inputMesh = mesh;
            setDefaultParams();
        }

        void QuadricEdgeCollapseResampling::setMesh(VC_MeshType::Pointer mesh) {
            _inputMesh = mesh;
        }

        void QuadricEdgeCollapseResampling::setDefaultParams(){
            _collapseParams.SetDefaultParams();
            _collapseParams.PreserveBoundary = true;
            _collapseParams.PreserveTopology = true;
        }


        void QuadricEdgeCollapseResampling::compute() {

            vcg::LocalOptimization<VC_MeshType::Pointer> DeciSession(_inputMesh, &_collapseParams);
            DeciSession.Init<CollapseType>();
            DeciSession.DoOptimization();
        }

        VC_MeshType::Pointer QuadricEdgeCollapseResampling::getMesh(){
            return _inputMesh;
        }
    }//meshing
}//volcart