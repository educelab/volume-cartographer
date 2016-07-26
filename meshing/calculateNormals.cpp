//
// Created by Hannah Hatch on 7/26/16.
//

#include "calculateNormals.h"


namespace volcart{
    namespace meshing{
        calculateNormals::calculateNormals()
        {
            _in_mesh = nullptr;
        }

        calculateNormals::calculateNormals(VC_MeshType::Pointer mesh)
        {
            _in_mesh = mesh;
            volcart::meshing::deepCopy(_in_mesh,_out_mesh);
            _vertex_normals = std::vector<cv::Vec3d>(_out_mesh->GetNumberOfPoints(),0);
        }

        void calculateNormals::setMesh(VC_MeshType::Pointer mesh)
        {
            _in_mesh = mesh;
            volcart::meshing::deepCopy(_in_mesh,_out_mesh);
            _vertex_normals = std::vector<cv::Vec3d>(_out_mesh->GetNumberOfPoints(),0);
        }

        void calculateNormals::computeNormals()
        {

            VC_PointsInCellIterator points;
            for(VC_CellIterator cellIterator = _in_mesh->GetCells()->Begin(); cellIterator!=_in_mesh->GetCells()->End(); cellIterator++)
            {
                cv::Vec3d v0, v1, v2, e0, e1;
                std::vector<unsigned long> point_ids;
                points = cellIterator->Value()->PointIdsBegin();
                VC_PointType point_C = _in_mesh->GetPoint(*points);
                v0[0] = point_C[0];
                v0[1] = point_C[1];
                v0[2] = point_C[2];
                point_ids.push_back(*points);

                points++;
                point_C = _in_mesh->GetPoint(*points);
                v1[0] = point_C[0];
                v1[1] = point_C[1];
                v1[2] = point_C[2];
                point_ids.push_back(*points);

                points++;
                point_C = _in_mesh->GetPoint(*points);
                v2[0] = point_C[0];
                v2[1] = point_C[1];
                v2[2] = point_C[2];
                point_ids.push_back(*points);

                e0 = v2 - v0;
                e1 = v1 - v0;

                cv::Vec3d normals;
                normals = e1.cross(e0);
                _vertex_normals[point_ids[0]] += normals;
                _vertex_normals[point_ids[1]] += normals;
                _vertex_normals[point_ids[2]] += normals;

            }//Cells loop


            unsigned long k = 0;

            for(VC_PointsInMeshIterator point =_in_mesh->GetPoints()->Begin(); point!= _in_mesh->GetPoints()->End(); point++, k++)
            {
                normalize(_vertex_normals, _vertex_normals);
                VC_PixelType pnt_normal;
                pnt_normal[0] = _vertex_normals[k][0];
                pnt_normal[1] = _vertex_normals[k][1];
                pnt_normal[2] = _vertex_normals[k][2];
                _out_mesh->SetPointData(k, pnt_normal);
            }//Points Loop
        }

        VC_MeshType::Pointer calculateNormals::getOutput()
        {
            return _out_mesh;
        }
    }//meshing
}//volcart