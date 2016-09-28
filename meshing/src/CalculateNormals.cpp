//
// Created by Hannah Hatch on 7/26/16.
//

#include "meshing/CalculateNormals.h"
#include "common/vc_defines.h"
#include "meshing/deepCopy.h"

using namespace volcart::meshing;

///// Construction /////
CalculateNormals::CalculateNormals(){};

CalculateNormals::CalculateNormals(MeshType::Pointer mesh)
{
    _input = mesh;
    _output = MeshType::New();
    volcart::meshing::deepCopy(_input, _output);
    _vertex_normals = std::vector<cv::Vec3d>(_output->GetNumberOfPoints(), 0);
}

///// Input/Output /////
void CalculateNormals::setMesh(MeshType::Pointer mesh)
{
    _input = mesh;
    _output = MeshType::New();
    volcart::meshing::deepCopy(_input, _output);
    _vertex_normals = std::vector<cv::Vec3d>(_output->GetNumberOfPoints(), 0);
}

volcart::MeshType::Pointer CalculateNormals::getMesh() const { return _output; }

///// Processing /////
void CalculateNormals::compute()
{
    _computeNormals();
    _assignToMesh();
}

void CalculateNormals::_computeNormals()
{

    for (auto c_it = _input->GetCells()->Begin();
         c_it != _input->GetCells()->End(); ++c_it) {

        // Empty vectors for the vertex and edge info
        cv::Vec3d v0, v1, v2, e0, e1;

        // Collect the point id's for this cell
        std::vector<unsigned long> p_ids;
        PointType vert;
        for (auto p = c_it->Value()->PointIdsBegin();
             p != c_it->Value()->PointIdsEnd(); ++p) {
            p_ids.push_back(*p);
        }

        // To-Do: Throw exception if p_ids.size() != 3

        // Collect the vertex info for each point
        vert = _input->GetPoint(p_ids[0]);
        v0(0) = vert[0];
        v0(1) = vert[1];
        v0(2) = vert[2];

        vert = _input->GetPoint(p_ids[1]);
        v1(0) = vert[0];
        v1(1) = vert[1];
        v1(2) = vert[2];

        vert = _input->GetPoint(p_ids[2]);
        v2(0) = vert[0];
        v2(1) = vert[1];
        v2(2) = vert[2];

        // Get the edge vectors
        e0 = v2 - v0;
        e1 = v1 - v0;

        // Take the cross-product
        cv::Vec3d normals;
        normals = e1.cross(e0);

        // Add the norm for this face to the running sum for each vertex
        _vertex_normals[p_ids[0]] += normals;
        _vertex_normals[p_ids[1]] += normals;
        _vertex_normals[p_ids[2]] += normals;

    }  // Cells loop
}

void CalculateNormals::_assignToMesh()
{

    for (auto point = _input->GetPoints()->Begin();
         point != _input->GetPoints()->End(); ++point) {
        cv::Vec3d norm = _vertex_normals[point.Index()];
        cv::normalize(norm, norm);

        PixelType pnt_normal;
        pnt_normal[0] = norm(0);
        pnt_normal[1] = norm(1);
        pnt_normal[2] = norm(2);

        _output->SetPointData(point.Index(), pnt_normal);

    }  // Points Loop
}
