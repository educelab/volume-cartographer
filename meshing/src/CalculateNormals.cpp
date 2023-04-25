#include "vc/meshing/CalculateNormals.hpp"

using namespace volcart;
using namespace volcart::meshing;

///// Input/Output /////
void CalculateNormals::setMesh(const ITKMesh::Pointer& mesh) { input_ = mesh; }

///// Processing /////
ITKMesh::Pointer CalculateNormals::compute()
{
    output_ = ITKMesh::New();
    DeepCopy(input_, output_);

    compute_normals_();
    assign_to_mesh_();

    return output_;
}

void CalculateNormals::compute_normals_()
{
    vertexNormals_ = std::vector<cv::Vec3d>(output_->GetNumberOfPoints(), 0);

    for (auto cellIt = input_->GetCells()->Begin();
         cellIt != input_->GetCells()->End(); ++cellIt) {

        // Empty vectors for the vertex and edge info
        cv::Vec3d v0, v1, v2, e0, e1;

        // Collect the point id's for this cell
        std::vector<uint64_t> pointIds;
        ITKPoint vert;
        for (auto p = cellIt->Value()->PointIdsBegin();
             p != cellIt->Value()->PointIdsEnd(); ++p) {
            pointIds.push_back(*p);
        }

        // To-Do: #185

        // Collect the vertex info for each point
        vert = input_->GetPoint(pointIds[0]);
        v0(0) = vert[0];
        v0(1) = vert[1];
        v0(2) = vert[2];

        vert = input_->GetPoint(pointIds[1]);
        v1(0) = vert[0];
        v1(1) = vert[1];
        v1(2) = vert[2];

        vert = input_->GetPoint(pointIds[2]);
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
        vertexNormals_[pointIds[0]] += normals;
        vertexNormals_[pointIds[1]] += normals;
        vertexNormals_[pointIds[2]] += normals;
    }
}

void CalculateNormals::assign_to_mesh_()
{
    for (auto point = input_->GetPoints()->Begin();
         point != input_->GetPoints()->End(); ++point) {
        cv::Vec3d norm = vertexNormals_[point.Index()];
        cv::normalize(norm, norm);

        output_->SetPointData(point.Index(), norm.val);
    }
}
