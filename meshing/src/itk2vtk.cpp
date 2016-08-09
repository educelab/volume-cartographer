//
// Created by Seth Parker on 8/3/15.
//

#include "meshing/itk2vtk.h"

namespace volcart {
    namespace meshing {

        ///// ITK Mesh -> VTK Polydata /////
        itk2vtk::itk2vtk( VC_MeshType::Pointer input, vtkPolyData* output ) {

            // points + normals
            vtkPoints *points = vtkPoints::New();
            vtkSmartPointer<vtkDoubleArray> pointNormals = vtkSmartPointer<vtkDoubleArray>::New();
            pointNormals->SetNumberOfComponents(3); //3d normals (ie x,y,z)
            pointNormals->SetNumberOfTuples(input->GetNumberOfPoints());

            for ( VC_PointsInMeshIterator point = input->GetPoints()->Begin(); point != input->GetPoints()->End(); ++point ) {
                // get the point's normal
                VC_PixelType normal;
                input->GetPointData(point.Index(), &normal);
                double ptNorm[3] = {normal[0], normal[1], normal[2]};

                // assign the values
                points->InsertPoint(point->Index(), point->Value()[0], point->Value()[1], point->Value()[2]);
                pointNormals->SetTuple(point->Index(), ptNorm);
            }


            // cells
            vtkCellArray *polys = vtkCellArray::New();
            for ( VC_CellIterator cell = input->GetCells()->Begin(); cell != input->GetCells()->End(); ++cell ) {

                vtkIdList *poly = vtkIdList::New();
                for ( VC_PointsInCellIterator point = cell.Value()->PointIdsBegin(); point != cell.Value()->PointIdsEnd(); ++point )
                    poly->InsertNextId(*point);

                polys->InsertNextCell(poly);
            }


            // assign to the mesh
            output->SetPoints(points);
            output->SetPolys(polys);
            output->GetPointData()->SetNormals(pointNormals);

        };

        ///// VTK Polydata -> ITK Mesh /////
        vtk2itk::vtk2itk( vtkPolyData* input, VC_MeshType::Pointer output ) {

            // points + normals
            vtkDataArray *pointNormals = input->GetPointData()->GetNormals();
            for ( vtkIdType p_id = 0; p_id < input->GetNumberOfPoints(); ++p_id ) {

                VC_PointType point = input->GetPoint(p_id);
                VC_PixelType normal = pointNormals->GetTuple(p_id);

                output->SetPoint(p_id, point);
                output->SetPointData(p_id, normal);
            }

            // cells
            VC_CellType::CellAutoPointer cell;
            for ( vtkIdType c_id = 0; c_id < input->GetNumberOfCells(); ++c_id ) {

                vtkCell *inputCell = input->GetCell(c_id); // input cell
                cell.TakeOwnership( new VC_TriangleType ); // output cell

                for ( vtkIdType p_id = 0; p_id < inputCell->GetNumberOfPoints(); ++p_id ) {
                    cell->SetPointId(p_id, inputCell->GetPointId(p_id)); // assign the point id's
                }

                output->SetCell( c_id, cell );
            }

        };

        ///// ITK Mesh -> ITK QuadEdge Mesh /////
        itk2itkQE::itk2itkQE(VC_MeshType::Pointer input, volcart::QuadMesh::Pointer output ) {

          // Vertices
          volcart::QuadPoint p;
          VC_PixelType n;
          for ( VC_PointsInMeshIterator point = input->GetPoints()->Begin(); point != input->GetPoints()->End(); ++point ) {
            p = point->Value();
            input->GetPointData(point->Index(), &n);

            output->SetPoint(point->Index(), p);
            output->SetPointData(point->Index(), n);
          }

          // Faces
          for ( VC_CellIterator cell = input->GetCells()->Begin(); cell != input->GetCells()->End(); ++cell ) {
            // Collect the point id's
            std::vector<QuadPointIdentifier> v_ids;
            for (VC_PointsInCellIterator point = cell.Value()->PointIdsBegin();
                 point != cell.Value()->PointIdsEnd(); ++point) {
              v_ids.push_back(*point);
            }
            //Assign to the mesh
            output->AddFaceTriangle(v_ids[0], v_ids[1], v_ids[2]);
          }
        }

        ///// ITK QuadEdge Mesh -> ITK Mesh /////
        itkQE2itk::itkQE2itk(volcart::QuadMesh::Pointer input, VC_MeshType::Pointer output) {

          // Vertices
          VC_PointType p;
          VC_PixelType n;
          for( auto point = input->GetPoints()->Begin(); point != input->GetPoints()->End(); ++point ) {
            p = point->Value();
            input->GetPointData(point->Index(), &n);

            output->SetPoint(point->Index(), p);
            output->SetPointData(point->Index(), n);
          }

          // Faces
          VC_CellType::CellAutoPointer cell;
          QuadCellIdentifier id = 0; // QE Meshes use a map so we have to reset their cell ids
          for ( QuadCellIterator c_it = input->GetCells()->Begin(); c_it != input->GetCells()->End(); ++c_it, ++id ) {
            cell.TakeOwnership( new VC_TriangleType ); // output cell
            cell->SetPointIds( c_it->Value()->PointIdsBegin(), c_it->Value()->PointIdsEnd() );

            output->SetCell( id, cell );
          }
        }

    } // namespace meshing
} // namespace volcart
