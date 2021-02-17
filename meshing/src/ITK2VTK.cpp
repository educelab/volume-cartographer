#include <vtkDoubleArray.h>
#include <vtkPointData.h>

#include "vc/meshing/ITK2VTK.hpp"

namespace volcart::meshing
{

///// ITK Mesh -> VTK Polydata /////
ITK2VTK::ITK2VTK(ITKMesh::Pointer input, vtkSmartPointer<vtkPolyData> output)
{

    // points + normals
    auto points = vtkSmartPointer<vtkPoints>::New();
    auto pointNormals = vtkSmartPointer<vtkDoubleArray>::New();
    pointNormals->SetNumberOfComponents(3);  // 3d normals (ie x,y,z)

    for (auto point = input->GetPoints()->Begin();
         point != input->GetPoints()->End(); ++point) {
        // assign the point
        points->InsertPoint(
            point->Index(), point->Value()[0], point->Value()[1],
            point->Value()[2]);

        // assign the normal
        ITKPixel normal;
        if (input->GetPointData(point.Index(), &normal)) {
            std::array<double, 3> ptNorm = {normal[0], normal[1], normal[2]};
            pointNormals->InsertTuple(point->Index(), ptNorm.data());
        }
    }

    // cells
    auto polys = vtkSmartPointer<vtkCellArray>::New();
    for (auto cell = input->GetCells()->Begin();
         cell != input->GetCells()->End(); ++cell) {

        auto poly = vtkSmartPointer<vtkIdList>::New();
        for (auto point = cell.Value()->PointIdsBegin();
             point != cell.Value()->PointIdsEnd(); ++point) {
            poly->InsertNextId(*point);
        }

        polys->InsertNextCell(poly);
    }

    // assign to the mesh
    output->SetPoints(points);
    output->SetPolys(polys);
    if (pointNormals->GetNumberOfTuples() > 0) {
        output->GetPointData()->SetNormals(pointNormals);
    }
}

///// VTK Polydata -> ITK Mesh /////
VTK2ITK::VTK2ITK(vtkSmartPointer<vtkPolyData> input, ITKMesh::Pointer output)
{

    // points + normals
    auto pointNormals = input->GetPointData()->GetNormals();
    for (vtkIdType pointId = 0; pointId < input->GetNumberOfPoints();
         ++pointId) {
        auto point = input->GetPoint(pointId);
        output->SetPoint(pointId, point);
        if (pointNormals != nullptr) {
            auto normal = pointNormals->GetTuple(pointId);
            output->SetPointData(pointId, normal);
        }
    }

    // cells
    ITKCell::CellAutoPointer cell;
    for (vtkIdType cellId = 0; cellId < input->GetNumberOfCells(); ++cellId) {
        auto inputCell = input->GetCell(cellId);  // input cell
        cell.TakeOwnership(new ITKTriangle);      // output cell

        for (vtkIdType pointId = 0; pointId < inputCell->GetNumberOfPoints();
             ++pointId) {
            cell->SetPointId(
                pointId,
                inputCell->GetPointId(pointId));  // assign the point id's
        }

        output->SetCell(cellId, cell);
    }
}

///// ITK Mesh -> ITK QuadEdge Mesh /////
ITK2ITKQE::ITK2ITKQE(ITKMesh::Pointer input, QuadEdgeMesh::Pointer output)
{
    // Vertices
    volcart::QuadPoint p;
    ITKPixel n;
    for (auto point = input->GetPoints()->Begin();
         point != input->GetPoints()->End(); ++point) {
        // Assign the point
        p = point->Value();
        output->SetPoint(point->Index(), p);

        // Assign the normal
        if (input->GetPointData(point->Index(), &n)) {
            output->SetPointData(point->Index(), n);
        }
    }

    // Faces
    for (auto cell = input->GetCells()->Begin();
         cell != input->GetCells()->End(); ++cell) {
        // Collect the point id's
        std::vector<QuadPointIdentifier> vIds;
        for (auto point = cell.Value()->PointIdsBegin();
             point != cell.Value()->PointIdsEnd(); ++point) {
            vIds.push_back(*point);
        }

        // Assign to the mesh
        output->AddFaceTriangle(vIds[0], vIds[1], vIds[2]);
    }
}

///// ITK QuadEdge Mesh -> ITK Mesh /////
ITKQE2ITK::ITKQE2ITK(QuadEdgeMesh::Pointer input, ITKMesh::Pointer output)
{
    // Vertices
    ITKPoint p;
    ITKPixel n;
    for (auto point = input->GetPoints()->Begin();
         point != input->GetPoints()->End(); ++point) {
        // Assign the point
        p = point->Value();
        output->SetPoint(point->Index(), p);

        // Assign the normal
        if (input->GetPointData(point->Index(), &n)) {
            output->SetPointData(point->Index(), n);
        }
    }

    // Faces
    ITKCell::CellAutoPointer cell;
    QuadCellIdentifier id =
        0;  // QE Meshes use a map so we have to reset their cell ids
    for (auto cellIt = input->GetCells()->Begin();
         cellIt != input->GetCells()->End(); ++cellIt, ++id) {
        cell.TakeOwnership(new ITKTriangle);  // output cell
        cell->SetPointIds(
            cellIt->Value()->PointIdsBegin(), cellIt->Value()->PointIdsEnd());

        output->SetCell(id, cell);
    }
}
}  // namespace volcart::meshing
