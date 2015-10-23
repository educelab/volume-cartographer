//
// Created by Seth Parker on 9/18/15.
//

#include "testingMesh.h"

namespace volcart {
namespace testing {

    ///// Constructor /////
    testingMesh::testingMesh() {

        // dimensions of the mesh plane
        int width = 5, height = 5;

        // generate the points along the y-axis
        double y = 0;
        for ( double x = 0; x < width; ++x) {
            for ( double z = 0; z < height; ++z ) {
                _add_vertex(x, y, z);
            }
        }

        // generate the cells
        for (int i = 1; i < height; ++i) {
            for (int j = 1; j < width; ++j) {
                int v1, v2, v3, v4;
                v1 = i * width + j;
                v2 = v1 - 1;
                v3 = v2 - width;
                v4 = v1 - width;
                _add_cell(v1, v2, v3);
                _add_cell(v1, v3, v4);
            }
        }
    }

    ///// Type Conversions /////
    // return an itk mesh
    VC_MeshType::Pointer testingMesh::itkMesh() {
        VC_MeshType::Pointer output = VC_MeshType::New();

        // points + normals
        VC_PointType point;
        VC_PixelType normal;
        for ( unsigned long p_id = 0; p_id < _points.size(); ++p_id ) {
            point[0]  = _points[p_id].x;
            point[1]  = _points[p_id].y;
            point[2]  = _points[p_id].z;
            normal[0] = _points[p_id].nx;
            normal[1] = _points[p_id].ny;
            normal[2] = _points[p_id].nz;

            output->SetPoint(p_id, point);
            output->SetPointData(p_id, normal);
        }

        // cells
        VC_CellType::CellAutoPointer cell;
        for ( unsigned long c_id = 0; c_id < _cells.size(); ++c_id ) {
            cell.TakeOwnership( new VC_TriangleType );
            cell->SetPointId( 0, _cells[c_id].v1 );
            cell->SetPointId( 1, _cells[c_id].v2 );
            cell->SetPointId( 2, _cells[c_id].v3 );
            output->SetCell( c_id, cell );
        }

        return output;
    }

    // initialize a vtk mesh //

    vtkPolyData* testingMesh::vtkMesh() {
        vtkPolyData* output = vtkPolyData::New();

        // points + normals
        vtkPoints *points = vtkPoints::New();
        vtkSmartPointer<vtkDoubleArray> pointNormals = vtkSmartPointer<vtkDoubleArray>::New();
        pointNormals->SetNumberOfComponents(3);
        pointNormals->SetNumberOfTuples( _points.size() );

        for ( unsigned long p_id = 0; p_id < _points.size(); ++p_id ) {

            //put normals for the current point in an array
            double ptNorm[3] = { _points[p_id].nx, _points[p_id].ny, _points[p_id].nz };

            //set the point and normal values for each point
            points->InsertPoint(p_id, _points[p_id].x, _points[p_id].y, _points[p_id].z);
            pointNormals->SetTuple(p_id, ptNorm);
        }

        // polys
        vtkCellArray *polys = vtkCellArray::New();
        for ( unsigned long c_id = 0; c_id < _cells.size(); ++c_id ){

            vtkIdList *poly = vtkIdList::New();
            poly->InsertNextId(_cells[c_id].v1);
            poly->InsertNextId(_cells[c_id].v2);
            poly->InsertNextId(_cells[c_id].v3);

            polys->InsertNextCell(poly);

        }

        //assign to the mesh
        output->SetPoints(points);
        output->SetPolys(polys);
        output->GetPointData()->SetNormals(pointNormals);

        return output;
    }


    ///// Mesh Generation Helper Functions /////
    void testingMesh::_add_vertex(double x, double y, double z) {
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

    void testingMesh::_add_cell(int v1, int v2, int v3) {
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
        magnitude = sqrt(nx*nx + ny*ny + nz*nz);
        nx /= magnitude;
        ny /= magnitude;
        nz /= magnitude;

        // update the vertex normals
        _update_normal(v1, nx, ny, nz);
        _update_normal(v2, nx, ny, nz);
        _update_normal(v3, nx, ny, nz);
    }

    void testingMesh::_update_normal(int vertex, double nx_in, double ny_in, double nz_in) {
        // recalculate average (unaverage, add new component, recalculate average)
        VC_Vertex v = _points[vertex];
        v.nx = (v.nx * v.face_count + nx_in) / (v.face_count + 1);
        v.ny = (v.ny * v.face_count + ny_in) / (v.face_count + 1);
        v.nz = (v.nz * v.face_count + nz_in) / (v.face_count + 1);
        v.face_count++;
        _points[vertex] = v;
    }


} // namespace testing
} // namespace volcart