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
VC_MeshType::Pointer ShapePrimitive::itkMesh()
{
    VC_MeshType::Pointer output = VC_MeshType::New();

    // points + normals
    VC_PointType point;
    VC_PixelType normal;
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
    VC_CellType::CellAutoPointer cell;
    for (unsigned long c_id = 0; c_id < _cells.size(); ++c_id) {
        cell.TakeOwnership(new VC_TriangleType);
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
volcart::OrderedPointSet<volcart::Point3d>ShapePrimitive::orderedPoints(bool noisify)
{

    volcart::OrderedPointSet<volcart::Point3d> output(_orderedWidth);
    std::vector<Point3d> temp_row;
    double offset = 0.0;
    if (noisify)
        offset = 5.0;
    int point_counter = 0;  // This is the worst. // SP
    int width_cnt = 0;
    for (auto p_id : _points) {
        volcart::Point3d point;
      if(width_cnt == output.width())
      {
          output.push_row(temp_row);
          temp_row.clear();
          width_cnt = 0;
      }

        point[0] = p_id.x;
        point[1] = p_id.y;

        if (noisify && (point_counter % 2 == 0)) {
            point[2] = p_id.z + offset;
            point[1] = p_id.z;  // added this to take the points out of the x-z
            // plane to test impact of mls
        } else
            point[2] = p_id.z;
        temp_row.push_back(point);
        ++point_counter;
        ++width_cnt;


    }
    return output;
}
volcart::PointSet<volcart::Point3d>ShapePrimitive::unOrderedPoints(bool noisify)
{

    volcart::PointSet<volcart::Point3d> output;
    double offset = 0.0;
    if (noisify)
    {
        offset = 5.0;
    }
    int point_counter = 0;  // This is the worst. // SP
    for (auto p_id : _points) {
        volcart::Point3d point;

            point[0] = p_id.x;
            point[1] = p_id.y;

            if (noisify && (point_counter % 2 == 0)) {
                point[2] = p_id.z + offset;
                point[1] = p_id.z;  // added this to take the points out of the x-z
                // plane to test impact of mls
            } else
                point[2] = p_id.z;
            ++point_counter;
        output.push_back(point);
    }

    return output;
}

// Return Point Cloud
volcart::OrderedPointSet<volcart::Point6d>ShapePrimitive::orderedPointNormal()
{

    volcart::OrderedPointSet<volcart::Point6d> output(_orderedWidth);
    std::vector<volcart::Point6d> temp_row;
    for (auto p_id :_points) {
        volcart::Point6d point;
        for(int i = 0; i < _orderedWidth; i++){
            point[0] = p_id.x;
            point[1] = p_id.y;
            point[2] = p_id.z;
            point[3] = p_id.nx;
            point[4] = p_id.ny;
            point[5] = p_id.nz;

            temp_row.push_back(point);
        }
        output.push_row(temp_row);

    }

    return output;
};

volcart::PointSet<volcart::Point6d> ShapePrimitive::unOrderedPointNormal()
{
    volcart::PointSet<volcart::Point6d> output;
    for (auto p_id : _points) {
        volcart::Point6d point;
            point[0] = p_id.x;
            point[1] = p_id.y;
            point[2] = p_id.z;
            point[3] = p_id.nx;
            point[4] = p_id.ny;
            point[5] = p_id.nz;
        output.push_back(point);

    }
    return output;
}


///// Mesh Generation Helper Functions /////
void ShapePrimitive::_add_vertex(double x, double y, double z)
{
    VC_Vertex v;
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
    VC_Cell f;
    f.v1 = v1;
    f.v2 = v2;
    f.v3 = v3;
    _cells.push_back(f);

    // calculate vertex normals (average of surface normals of each triangle)
    // get surface normal of this triangle
    double nx, ny, nz, vx, vy, vz, wx, wy, wz, magnitude;

    VC_Vertex vt1 = _points[v1];
    VC_Vertex vt2 = _points[v2];
    VC_Vertex vt3 = _points[v3];

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
    VC_Vertex v = _points[vertex];
    v.nx = (v.nx * v.face_count + nx_in) / (v.face_count + 1);
    v.ny = (v.ny * v.face_count + ny_in) / (v.face_count + 1);
    v.nz = (v.nz * v.face_count + nz_in) / (v.face_count + 1);
    v.face_count++;
    _points[vertex] = v;
}

}  // namespace shapes
}  // namespace volcart
