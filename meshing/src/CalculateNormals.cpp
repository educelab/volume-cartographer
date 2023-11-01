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

    // Compute the mesh centroid to use as reference when deciding if the mesh
    // normals are flipped. Should produce consistent result on convex meshes.
    cv::Vec3d meshCentroid = compute_mesh_centroid_();

    std::size_t outwardNormalsCount = 0;
    std::size_t inwardNormalsCount = 0;

    for (auto cellIt = input_->GetCells()->Begin();
         cellIt != input_->GetCells()->End(); ++cellIt) {

        // Collect the point id's for this cell
        std::vector<uint64_t> pointIds;
        ITKPoint vert;
        for (auto p = cellIt->Value()->PointIdsBegin();
             p != cellIt->Value()->PointIdsEnd(); ++p) {
            pointIds.push_back(*p);
        }

        // To-Do: #185

        cv::Vec3d faceCentroid(0.0, 0.0, 0.0);

        // Collect the vertex info for each point
        vert = input_->GetPoint(pointIds[0]);
        cv::Vec3d v0(vert[0], vert[1], vert[2]);
        faceCentroid += v0;

        vert = input_->GetPoint(pointIds[1]);
        cv::Vec3d v1(vert[0], vert[1], vert[2]);
        faceCentroid += v1;

        vert = input_->GetPoint(pointIds[2]);
        cv::Vec3d v2(vert[0], vert[1], vert[2]);
        faceCentroid += v2;

        // Centroid is the average of the three points of the face.
        faceCentroid /= 3.;

        // Get the edge vectors
        cv::Vec3d e0 = v2 - v0;
        cv::Vec3d e1 = v1 - v0;

        // Take the cross-product
        cv::Vec3d normals;
        normals = e1.cross(e0);

        // Accumulate the relative direction of the normals for this face.
        if (normals.dot(faceCentroid - meshCentroid) > 0) {
            outwardNormalsCount++;
        } else {
            inwardNormalsCount++;
        }

        // Add the norm for this face to the running sum for each vertex
        vertexNormals_[pointIds[0]] += normals;
        vertexNormals_[pointIds[1]] += normals;
        vertexNormals_[pointIds[2]] += normals;
    }

    // If more face normals were facing outwards than inwards, they are flipped.
    flippedNormals_ = outwardNormalsCount > inwardNormalsCount;
}

void CalculateNormals::assign_to_mesh_()
{
    for (auto point = input_->GetPoints()->Begin();
         point != input_->GetPoints()->End(); ++point) {
        cv::Vec3d norm = vertexNormals_[point.Index()];
        cv::normalize(norm, norm);

        if (shouldOrientNormals && flippedNormals_) {
            norm = -norm;
        }
        output_->SetPointData(point.Index(), norm.val);
    }
}

cv::Vec3d CalculateNormals::compute_mesh_centroid_() {
    cv::Vec3d meshCentroid(0.0, 0.0, 0.0);
    double pointCount = 1.0;
    for (auto it = input_->GetPoints()->Begin(); it != input_->GetPoints()->End(); ++it) {
        auto vert = it.Value();
        cv::Vec3d point(vert[0], vert[1], vert[2]);
        meshCentroid = meshCentroid + (point - meshCentroid)/pointCount;
        pointCount += 1.0;
    }
    return meshCentroid;
}
