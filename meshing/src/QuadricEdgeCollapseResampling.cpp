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
//                     unsigned long temp = vertex_id[*p_id] ;
                     ivp[i] = &(input.vert[*p_id]);
                      //vi++;
                     // if std::vector<vertex> vs, then vs[p_it]

                 }
                fi->V(0)=ivp[0];
                fi->V(1)=ivp[1];
                fi->V(2)=ivp[2];

                fi++;
            }

//            float TargetError = std::numeric_limits<float>::max();
            vcg::LocalOptimization<MyMesh> DeciSession(input, &_collapseParams);
            DeciSession.SetTargetSimplices(iterations);
//            DeciSession.SetTargetMetric(TargetError);
            DeciSession.Init<MyTriEdgeCollapse>();
            while(input.fn > iterations && DeciSession.DoOptimization())
            {
                std::cout << "Current mesh is " << input.fn << std::endl;
            }
            DeciSession.Finalize<MyTriEdgeCollapse>();
        }

        VC_MeshType::Pointer QuadricEdgeCollapseResampling::getMesh(){
            _outputMesh =  VC_MeshType::New();
            VC_PointType point;
            unsigned long j = 0;
            MyMesh::VertexPointer vp;
            vcg::SimpleTempData<MyMesh::VertContainer , unsigned long> indices(input.vert);
           // MyMesh::PerVertexAttributeHandle<unsigned long> vertex_id = vcg::tri::Allocator<MyMesh>::GetPerVertexAttribute<unsigned long>(input, std::string("Vertex ID"));
            for(auto vi = input.vert.begin(); vi!=input.vert.end(); vi++){
                vp=&(*vi);
                indices[vp] = j;
                if(!vi->IsD()){
                       point[0] = vi->P()[0];
                       point[1] = vi->P()[1];
                       point[2] = vi->P()[2];

                    _outputMesh->SetPoint(j, point);
                    j++;
                   }
            }
            unsigned long cell_cnt = 0;
            VC_CellType::CellAutoPointer newCell;
            for(auto fi = input.face.begin(); fi!=input.face.end(); fi++){
                if(!fi->IsD())
                {   newCell.TakeOwnership(new VC_TriangleType);

                    unsigned long point1 = indices[fi->V(0)];
                    unsigned long point2 = indices[fi->V(1)];
                    unsigned long point3 = indices[fi->V(2)];


                    newCell->SetPointId(0, point1);
                    newCell->SetPointId(1, point2);
                    newCell->SetPointId(2, point3);

                    _outputMesh->SetCell(cell_cnt, newCell);
                    cell_cnt++;
                }

            }
            return _outputMesh;
        }
    }//meshing
}//volcart