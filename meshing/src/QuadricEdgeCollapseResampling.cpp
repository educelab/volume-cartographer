//
// Created by Hannah Hatch on 8/12/16.
//

#include "meshing/QuadricEdgeCollapseResampling.h"




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
           _collapseParams.NormalCheck = true;
            _collapseParams.NormalThrRad = M_PI/4.0;
            _collapseParams.QualityThr = 1;

        }


        void QuadricEdgeCollapseResampling::compute(int iterations) {
            MyMesh::FaceIterator fi = vcg::tri::Allocator<MyMesh>::AddFaces(input,_inputMesh.GetPointer()->GetNumberOfCells());
            vertex_id = vcg::tri::Allocator<MyMesh>::GetPerVertexAttribute<unsigned long>(input, std::string("Vertex ID"));
            //MyMesh::PerVertexAttributeHandle<unsigned long> vertex_id = vcg::tri::Allocator<MyMesh>::GetPerVertexAttribute<unsigned long>(input, std::string("Vertex ID"));
            unsigned long counter = 0;
            // Iterate over itk vertices and vcg::tri::Allocator<MyMesh>::AddVertex(m,MyMesh::CoordType ( 1.0, 1.0, 0.0));
            for(auto pointsIterator = _inputMesh->GetPoints()->Begin(); pointsIterator!=_inputMesh->GetPoints()->End(); pointsIterator++, counter++)
            {
                vcg::tri::Allocator<MyMesh>::AddVertex(input,MyMesh::CoordType(pointsIterator.Value()[0], pointsIterator.Value()[1], pointsIterator.Value()[2]));
                vertex_id[counter] = counter;
            }
            MyMesh::VertexIterator vi = input.vert.begin();
            for(VC_CellIterator cellIterator = _inputMesh->GetCells()->Begin(); cellIterator != _inputMesh->GetCells()->End(); cellIterator++)
            {
                int i = 0;
                MyMesh::VertexPointer ivp[3];
                for(auto p_id = cellIterator.Value()->PointIdsBegin(); p_id != cellIterator.Value()->PointIdsEnd(); p_id++, i++)
                 {
                     unsigned long temp = vertex_id[*p_id] ;
                     ivp[i] = &(input.vert[temp]);
                      //vi++;
                     // if std::vector<vertex> vs, then vs[p_it]

                 }
                fi->V(0)=ivp[0];
                fi->V(1)=ivp[1];
                fi->V(2)=ivp[2];

                fi++;
            }

            vcg::LocalOptimization<MyMesh> DeciSession(input, &_collapseParams);
            DeciSession.SetTargetSimplices(iterations);
            DeciSession.Init<MyTriEdgeCollapse>();
            DeciSession.DoOptimization();
            DeciSession.Finalize<MyTriEdgeCollapse>();
        }

        VC_MeshType::Pointer QuadricEdgeCollapseResampling::getMesh(){
            VC_PointType point;
            unsigned long point_cnt = 0;
           // MyMesh::PerVertexAttributeHandle<unsigned long> vertex_id = vcg::tri::Allocator<MyMesh>::GetPerVertexAttribute<unsigned long>(input, std::string("Vertex ID"));
            for(int i = 0; i<input.VN(); i++,point_cnt++){
                point[0] = input.vert[i].P()[0];
                point[1] = input.vert[i].P()[1];
                point[2] = input.vert[i].P()[2];

                _outputMesh->SetPoint(point_cnt, point);
            }
            unsigned long cell_cnt = 0;
            VC_CellType::CellAutoPointer newCell;
            for(auto fi = input.face.begin(); fi!=input.face.end(); fi++, cell_cnt++){
                newCell.TakeOwnership(new VC_TriangleType);

                unsigned long point1 = vertex_id[fi->V(0)];
                unsigned long point2 = vertex_id[fi->V(1)];
                unsigned long point3 = vertex_id[fi->V(2)];


                newCell->SetPointId(0, point1);
                newCell->SetPointId(1, point2);
                newCell->SetPointId(2, point3);

                _outputMesh->SetCell(cell_cnt, newCell);

            }
            return _outputMesh;
        }
    }//meshing
}//volcart