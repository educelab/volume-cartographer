//
// Created by Hannah Hatch on 8/12/16.
//

#include "meshing/QuadricEdgeCollapseResampling.h"

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
class MyMesh : public vcg::tri::TriMesh<std::vector<MyVertex>, std::vector<MyFace>, std::vector<MyEdge> > {};


typedef vcg::tri::BasicVertexPair<MyVertex> VertexPair;
class MyTriEdgeCollapse: public vcg::tri::TriEdgeCollapseQuadric<MyMesh, VertexPair, MyTriEdgeCollapse, vcg::tri::QInfoStandard<MyVertex> > {
public:
    typedef vcg::tri::TriEdgeCollapseQuadric<MyMesh,VertexPair,MyTriEdgeCollapse,vcg::tri::QInfoStandard<MyVertex> > TECQ;
    typedef MyMesh::VertexType::EdgeType EdgeType;
    inline MyTriEdgeCollapse(const VertexPair &p, int i, vcg::BaseParameterClass *pp) : TECQ(p,i,pp){}
};
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
            MyMesh input;
            MyMesh::VertexIterator vi = vcg::tri::Allocator<MyMesh>::AddVertices(input,_inputMesh.GetPointer()->GetNumberOfPoints());
            MyMesh::FaceIterator fi = vcg::tri::Allocator<MyMesh>::AddFaces(input,_inputMesh.GetPointer()->GetNumberOfCells());
            int i = 0;
            MyMesh::VertexPointer ivp[_inputMesh.GetPointer()->GetNumberOfPoints()];
            for(VC_CellIterator cellIterator = _inputMesh->GetCells()->Begin(); cellIterator != _inputMesh->GetCells()->End(); cellIterator++)
            {
                for(VC_PointsInCellIterator pointsIterator = cellIterator.Value()->PointIdsBegin(); pointsIterator!=cellIterator.Value()->PointIdsEnd(); pointsIterator++,i++)
                 {
                     ivp[i] =&*vi; vi->P()=MyMesh::CoordType (pointsIterator[0],pointsIterator[1],pointsIterator[2]); ++vi;
                 }
                fi->V(0)=ivp[i-2];
                fi->V(1)=ivp[i-1];
                fi->V(2)=ivp[i];
            }
            vcg::LocalOptimization<MyMesh> DeciSession(input, &_collapseParams);
            DeciSession.Init<MyTriEdgeCollapse>();
            DeciSession.DoOptimization();
        }

        VC_MeshType::Pointer QuadricEdgeCollapseResampling::getMesh(){
            return _inputMesh;
        }
    }//meshing
}//volcart