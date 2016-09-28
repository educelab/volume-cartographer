//
// Created by Seth Parker on 9/18/15.
//

#include "common/shapes/ShapePrimitive.h"

namespace volcart
{
namespace shapes
{

///// Type Conversions /////
// return an itk mesh
MeshType::Pointer ShapePrimitive::itkMesh()
{
    MeshType::Pointer output = MeshType::New();

    // points + normals
    PointType point;
    PixelType normal;
    for (unsigned long p_id = 0; p_id < _points.size(); ++p_id) {
        point[0] = _points[p_id].x;
        point[1] = _points[p_id].y;
        point[2] = _points[p_id].z;
        normal[0] = _points[p_id].nx;
        normal[1] = _points[p_id].ny;
        normal[2] = _points[p_id].nz;

        output->SetPoint(p_id, point);
        output->SetPointData(p_id, normal);
    }

    // cells
    CellType::CellAutoPointer cell;
    for (unsigned long c_id = 0; c_id < _cells.size(); ++c_id) {
        cell.TakeOwnership(new TriangleType);
        cell->SetPointId(0, _cells[c_id].v1);
        cell->SetPointId(1, _cells[c_id].v2);
        cell->SetPointId(2, _cells[c_id].v3);
        output->SetCell(c_id, cell);
    }

    return output;
}

// initialize a vtk mesh //

vtkSmartPointer<vtkPolyData> ShapePrimitive::vtkMesh()
{

    // construct new pointer to output mesh
    vtkSmartPointer<vtkPolyData> output = vtkSmartPointer<vtkPolyData>::New();

    // points + normals
    vtkPoints *points = vtkPoints::New();
    vtkSmartPointer<vtkDoubleArray> pointNormals =
        vtkSmartPointer<vtkDoubleArray>::New();
    pointNormals->SetNumberOfComponents(3);
    pointNormals->SetNumberOfTuples(_points.size());

    for (unsigned long p_id = 0; p_id < _points.size(); ++p_id) {

        // put normals for the current point in an array
        double ptNorm[3] = {_points[p_id].nx, _points[p_id].ny,
                            _points[p_id].nz};

        // set the point and normal values for each point
        points->InsertPoint(p_id, _points[p_id].x, _points[p_id].y,
                            _points[p_id].z);
        pointNormals->SetTuple(p_id, ptNorm);
    }

    // polys
    vtkCellArray *polys = vtkCellArray::New();
    for (unsigned long c_id = 0; c_id < _cells.size(); ++c_id) {

        vtkIdList *poly = vtkIdList::New();
        poly->InsertNextId(_cells[c_id].v1);
        poly->InsertNextId(_cells[c_id].v2);
        poly->InsertNextId(_cells[c_id].v3);

        polys->InsertNextCell(poly);
    }

    // assign to the mesh
    output->SetPoints(points);
    output->SetPolys(polys);
    output->GetPointData()->SetNormals(pointNormals);

    return output;
}

// Return Point Cloud
pcl::PointCloud<pcl::PointXYZ> ShapePrimitive::pointCloudXYZ(bool noisify)
{

    pcl::PointCloud<pcl::PointXYZ> output;

    double offset = 0.0;
    if (noisify)
        offset = 5.0;

    int point_counter = 0;  // This is the worst. // SP
    for (auto p_id = _points.begin(); p_id != _points.end(); ++p_id) {
        pcl::PointXYZ point;
        point.x = p_id->x;
        point.y = p_id->y;

        if (noisify && (point_counter % 2 == 0)) {
            point.z = p_id->z + offset;
            point.y = p_id->z;  // added this to take the points out of the x-z
                                // plane to test impact of mls
        } else
            point.z = p_id->z;

        output.push_back(point);
        ++point_counter;
    }

    // Set ordering information
    if (_orderedPoints) {
        output.width = _orderedWidth;
        output.height = _orderedHeight;
        output.resize(output.width * output.height);
    }

    return output;
}

// Return Point Cloud
pcl::PointCloud<pcl::PointXYZRGB> ShapePrimitive::pointCloudXYZRGB()
{

    pcl::PointCloud<pcl::PointXYZRGB> output;

    for (auto p_id = _points.begin(); p_id != _points.end(); ++p_id) {

        pcl::PointXYZRGB point;

        // Assign Point Values
        point.x = p_id->x;
        point.y = p_id->y;
        point.z = p_id->z;

        // assign color values
        // this is just for setting up testing values
        // values must fall within 0-255 range
        point.r = std::abs(p_id->x) + p_id->z;
        point.g = 35 * std::abs(p_id->x) + p_id->z;
        point.b = std::abs(p_id->x) + p_id->z * 45;

        output.push_back(point);
    }

    // Set ordering information
    if (_orderedPoints) {
        output.width = _orderedWidth;
        output.height = _orderedHeight;
        output.resize(output.width * output.height);
    }

    return output;
}

// Return Point Cloud
pcl::PointCloud<pcl::PointNormal> ShapePrimitive::pointCloudNormal()
{

    pcl::PointCloud<pcl::PointNormal> output;

    for (auto p_id = _points.begin(); p_id != _points.end(); ++p_id) {
        pcl::PointNormal point;
        point.x = p_id->x;
        point.y = p_id->y;
        point.z = p_id->z;
        point.normal_x = p_id->nx;
        point.normal_y = p_id->ny;
        point.normal_z = p_id->nz;

        output.push_back(point);
    }

    // Set ordering information
    if (_orderedPoints) {
        output.width = _orderedWidth;
        output.height = _orderedHeight;
        output.resize(output.width * output.height);
    }

    return output;
};

///// Mesh Generation Helper Functions /////
void ShapePrimitive::_add_vertex(double x, double y, double z)
{
    Vertex v;
    v.x = x;
    v.y = y;
    v.z = z;
    v.nx = 0;
    v.ny = 0;
    v.nz = 0;
    v.face_count = 0;
    _points.push_back(v);
}

void ShapePrimitive::_add_cell(int v1, int v2, int v3)
{
    Cell f;
    f.v1 = v1;
    f.v2 = v2;
    f.v3 = v3;
    _cells.push_back(f);

    // calculate vertex normals (average of surface normals of each triangle)
    // get surface normal of this triangle
    double nx, ny, nz, vx, vy, vz, wx, wy, wz, magnitude;

    Vertex vt1 = _points[v1];
    Vertex vt2 = _points[v2];
    Vertex vt3 = _points[v3];

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
    magnitude = sqrt(nx * nx + ny * ny + nz * nz);
    nx /= magnitude;
    ny /= magnitude;
    nz /= magnitude;

    // update the vertex normals
    _update_normal(v1, nx, ny, nz);
    _update_normal(v2, nx, ny, nz);
    _update_normal(v3, nx, ny, nz);
}

void ShapePrimitive::_update_normal(int vertex,
                                    double nx_in,
                                    double ny_in,
                                    double nz_in)
{
    // recalculate average (unaverage, add new component, recalculate average)
    Vertex v = _points[vertex];
    v.nx = (v.nx * v.face_count + nx_in) / (v.face_count + 1);
    v.ny = (v.ny * v.face_count + ny_in) / (v.face_count + 1);
    v.nz = (v.nz * v.face_count + nz_in) / (v.face_count + 1);
    v.face_count++;
    _points[vertex] = v;
}

}  // namespace shapes
}  // namespace volcart
