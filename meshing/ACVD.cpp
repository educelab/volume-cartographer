//
// Created by Media Team on 9/16/15.
//

#include "ACVD.h"

namespace volcart {
	namespace meshing {
		VC_MeshType::Pointer ACVD ( VC_MeshType::Pointer  inputMesh,
                                		int               NumberOfSamples,
                                		float             Gradation,
                                		int 							ConsoleOutput,
                                		int 							SubsamplingThreshold) {
			// Convert the itk mesh to a vtk mesh
    	vtkPolyData *vtkMesh = vtkPolyData::New();
    	volcart::meshing::itk2vtk(inputMesh, vtkMesh);

	    // ACVD's vtkSurface class used for resampling
  	  vtkSurface *Mesh = vtkSurface::New();
	    Mesh->CreateFromPolyData( vtkMesh );
    	Mesh->GetCellData()->Initialize();
    	Mesh->GetPointData()->Initialize();

    	// initialize paramaters needed for ACVD resampling 
    	int QuadricsOptimizationLevel = 1;

    	vtkIsotropicDiscreteRemeshing *Remesh = vtkIsotropicDiscreteRemeshing::New();

    	Remesh->SetInput(Mesh);
    	Remesh->SetFileLoadSaveOption(0);
    	Remesh->SetNumberOfClusters(NumberOfSamples);
    	Remesh->SetConsoleOutput(ConsoleOutput);
    	Remesh->SetSubsamplingThreshold(SubsamplingThreshold);
    	Remesh->GetMetric()->SetGradation(Gradation);
    	Remesh->Remesh();

    	// code from ACVD.cxx example, optimization of resampled mesh
    	// This is the iterative process discussed in:
    	//		Valette, Sébastien, and Jean‐Marc Chassery. "Approximated centroidal 
    	//		voronoi diagrams for uniform polygonal mesh coarsening." Computer 
    	//		Graphics Forum. Vol. 23. No. 3. Blackwell Publishing, Inc, 2004. 
    	// that repeats until the areas of each region (triangle) are approximately the same
    	if (QuadricsOptimizationLevel != 0) {
        // Note : this is an adaptation of Siggraph 2000 Paper :
        // Out-of-core simplification of large polygonal models
        vtkIntArray *Clustering = Remesh->GetClustering();

        int Cluster,NumberOfMisclassedItems = 0;

        double **ClustersQuadrics = new double*[NumberOfSamples];
        for (int i = 0; i < NumberOfSamples; i++) {
          ClustersQuadrics[i] = new double[9];
          for (int j = 0; j < 9; j++) {
            ClustersQuadrics[i][j] = 0;
          }
        }
        vtkIdList *FList = vtkIdList::New();
        for (int i = 0; i < Remesh->GetNumberOfItems (); i++) {
          Cluster = Clustering->GetValue (i);
          if ((Cluster >= 0)&& (Cluster < NumberOfSamples)) {
            if (Remesh->GetClusteringType() == 0) {
              vtkQuadricTools::AddTriangleQuadric(
                ClustersQuadrics[Cluster], Remesh->GetInput(), i, false);
            } else {
              Remesh->GetInput()->GetVertexNeighbourFaces(i, FList);
              for (int j = 0;j < FList->GetNumberOfIds(); j++)
                vtkQuadricTools::AddTriangleQuadric(
                  ClustersQuadrics[Cluster], Remesh->GetInput(), FList->GetId(j), false);
            }
          } else {
            NumberOfMisclassedItems++;
          }
        }
        FList->Delete();

        if (NumberOfMisclassedItems) {
          cout << NumberOfMisclassedItems << " Items with wrong cluster association" << endl;
        }

        double P[3];
        for (int i = 0; i < NumberOfSamples; i++) {
          Remesh->GetOutput()->GetPoint (i, P);
          vtkQuadricTools::ComputeRepresentativePoint(ClustersQuadrics[i], P, QuadricsOptimizationLevel);
          Remesh->GetOutput()->SetPointCoordinates (i, P);
          delete[] ClustersQuadrics[i];
        }
        delete [] ClustersQuadrics;

        Mesh->GetPoints()->Modified ();

        cout << "After Quadrics Post-processing : " << endl;
        Remesh->GetOutput()->DisplayMeshProperties();

    	}

    	vtkSmartPointer<vtkPolyDataNormals> normalGenerator = vtkSmartPointer<vtkPolyDataNormals>::New();
    	normalGenerator->SetInputData(Remesh->GetOutput());
    	normalGenerator->ComputePointNormalsOn();
    	normalGenerator->ComputeCellNormalsOn();
    	normalGenerator->Update();

    	//convert vtk mesh to itk mesh
    	VC_MeshType::Pointer  outputMesh = VC_MeshType::New();
    	volcart::meshing::vtk2itk(normalGenerator->GetOutput(), outputMesh);

    	Remesh->Delete();
    	Mesh->Delete();

    	return outputMesh;
		} // ACVD
	} // namespace meshing
} // namespace volcart