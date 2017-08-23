#include "vc/core/shapes/ShapePrimitive.hpp"

#include <array>

using namespace volcart;
using namespace volcart::shapes;

ShapePrimitive::ShapePrimitive()
    : ordered_(false), orderedWidth_(0), orderedHeight_(0)
{
}

///// Type Conversions /////
// return an itk mesh
ITKMesh::Pointer ShapePrimitive::itkMesh()
{
    auto output = ITKMesh::New();

    // points + normals
    ITKPoint point;
    ITKPixel normal;
    for (size_t pId = 0; pId < points_.size(); ++pId) {
        point[0] = points_[pId].x;
        point[1] = points_[pId].y;
        point[2] = points_[pId].z;
        normal[0] = points_[pId].nx;
        normal[1] = points_[pId].ny;
        normal[2] = points_[pId].nz;

        output->SetPoint(pId, point);
        output->SetPointData(pId, normal);
    }

    // cells
    ITKCell::CellAutoPointer cell;
    for (size_t cId = 0; cId < cells_.size(); ++cId) {
        cell.TakeOwnership(new ITKTriangle);
        cell->SetPointId(0, cells_[cId].v1);
        cell->SetPointId(1, cells_[cId].v2);
        cell->SetPointId(2, cells_[cId].v3);
        output->SetCell(cId, cell);
    }

    return output;
}

// initialize a vtk mesh //

vtkSmartPointer<vtkPolyData> ShapePrimitive::vtkMesh()
{

    // construct new pointer to output mesh
    auto output = vtkSmartPointer<vtkPolyData>::New();

    // points + normals
    vtkPoints* points = vtkPoints::New();
    auto pointNormals = vtkSmartPointer<vtkDoubleArray>::New();
    pointNormals->SetNumberOfComponents(3);
    pointNormals->SetNumberOfTuples(points_.size());

    for (size_t pId = 0; pId < points_.size(); ++pId) {

        // put normals for the current point in an array
        std::array<double, 3> ptNorm = {points_[pId].nx, points_[pId].ny,
                                        points_[pId].nz};

        // set the point and normal values for each point
        points->InsertPoint(
            pId, points_[pId].x, points_[pId].y, points_[pId].z);
        pointNormals->SetTuple(pId, ptNorm.data());
    }

    // polys
    vtkCellArray* polys = vtkCellArray::New();
    for (auto cell : cells_) {
        vtkIdList* poly = vtkIdList::New();
        poly->InsertNextId(cell.v1);
        poly->InsertNextId(cell.v2);
        poly->InsertNextId(cell.v3);
        polys->InsertNextCell(poly);
    }

    // assign to the mesh
    output->SetPoints(points);
    output->SetPolys(polys);
    output->GetPointData()->SetNormals(pointNormals);

    return output;
}

// Return point set (unordered)
volcart::PointSet<cv::Vec3d> ShapePrimitive::unorderedPoints()
{
    volcart::PointSet<cv::Vec3d> output;

    for (auto& p : points_) {
        output.push_back({p.x, p.y, p.z});
    }

    return output;
}

// Return point set + normals (unordered)
volcart::PointSet<cv::Vec6d> ShapePrimitive::unorderedPointNormal()
{
    volcart::PointSet<cv::Vec6d> output;
    for (auto p : points_) {
        output.push_back({p.x, p.y, p.z, p.nx, p.ny, p.nz});
    }
    return output;
}

// Return point set (ordered)
volcart::OrderedPointSet<cv::Vec3d> ShapePrimitive::orderedPoints()
{
    if (!ordered_) {
        throw std::runtime_error("Shape vertices not ordered");
    }

    volcart::OrderedPointSet<cv::Vec3d> output{orderedWidth_};
    std::vector<cv::Vec3d> tempRow;

    for (auto& p : points_) {

        if (tempRow.size() == output.width()) {
            output.pushRow(tempRow);
            tempRow.clear();
        }

        tempRow.emplace_back(p.x, p.y, p.z);
    }
    return output;
}

// Return point set + normals (ordered)
volcart::OrderedPointSet<cv::Vec6d> ShapePrimitive::orderedPointNormal()
{
    if (!ordered_) {
        throw std::runtime_error("Shape vertices not ordered");
    }

    volcart::OrderedPointSet<cv::Vec6d> output{orderedWidth_};
    std::vector<cv::Vec6d> tempRow;
    for (auto p : points_) {
        for (size_t i = 0; i < orderedWidth_; i++) {
            tempRow.emplace_back(p.x, p.y, p.z, p.nx, p.ny, p.nz);
        }
        output.pushRow(tempRow);
    }

    return output;
}

///// Mesh Generation Helper Functions /////
void ShapePrimitive::addVertex_(double x, double y, double z)
{
    SimpleMesh::Vertex v;
    v.x = x;
    v.y = y;
    v.z = z;
    v.nx = 0;
    v.ny = 0;
    v.nz = 0;
    v.faceCount = 0;
    points_.push_back(v);
}

void ShapePrimitive::addCell_(int v1, int v2, int v3)
{
    SimpleMesh::Cell f;
    f.v1 = v1;
    f.v2 = v2;
    f.v3 = v3;
    cells_.push_back(f);

    // calculate vertex normals (average of surface normals of each triangle)
    // get surface normal of this triangle
    double nx, ny, nz, vx, vy, vz, wx, wy, wz, magnitude;

    SimpleMesh::Vertex vt1 = points_[v1];
    SimpleMesh::Vertex vt2 = points_[v2];
    SimpleMesh::Vertex vt3 = points_[v3];

    vx = vt2.x - vt1.x;
    vy = vt2.y - vt1.y;
    vz = vt2.z - vt1.z;

    wx = vt3.x - vt1.x;
    wy = vt3.y - vt1.y;
    wz = vt3.z - vt1.z;

    nx = (vy * wz) - (vz * wy);
    ny = (vz * wx) - (vx * wz);
    nz = (vx * wy) - (vy * wx);

    // normalize
    magnitude = std::sqrt(nx * nx + ny * ny + nz * nz);
    nx /= magnitude;
    ny /= magnitude;
    nz /= magnitude;

    // update the vertex normals
    updateNormal_(v1, nx, ny, nz);
    updateNormal_(v2, nx, ny, nz);
    updateNormal_(v3, nx, ny, nz);
}

void ShapePrimitive::updateNormal_(int vertID, double nx, double ny, double nz)
{
    // recalculate average (unaverage, add new component, recalculate average)
    SimpleMesh::Vertex v = points_[vertID];
    v.nx = (v.nx * v.faceCount + nx) / (v.faceCount + 1);
    v.ny = (v.ny * v.faceCount + ny) / (v.faceCount + 1);
    v.nz = (v.nz * v.faceCount + nz) / (v.faceCount + 1);
    v.faceCount++;
    points_[vertID] = v;
}
