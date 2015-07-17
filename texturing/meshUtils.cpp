// meshUtils.h
// Abigail Coleman June 2015

#include "meshUtils.h"

VC_MeshType::Pointer smoothNormals ( VC_MeshType::Pointer  inputMesh,
                                     double                smoothingFactor ) {

    // declare pointer to new Mesh object to be returned
    VC_MeshType::Pointer  outputMesh = VC_MeshType::New();
    outputMesh = inputMesh; // copy faces, points, and old normals from input mesh

    // Define iterators
    VC_PointsInMeshIterator currentPoint, neighborPoint;
    currentPoint = inputMesh->GetPoints()->Begin();
    VC_PointsInMeshIterator pointsEnd = inputMesh->GetPoints()->End();
    
    // Variables for normal smoothing
    // cv::Vec3d neighborAvg;
    std::vector< long double > neighborAvg ( 3, 0 );
    double neighborCount, distance, pointID;

    // Use pointsLocator to find neighborhood within given radius
    typename VC_PointsLocatorType::Pointer pointsLocator = VC_PointsLocatorType::New();
    pointsLocator->SetPoints( inputMesh->GetPoints() );
    pointsLocator->Initialize();
    typename VC_PointsLocatorType::NeighborsIdentifierType neighborhood;

    // Iterate over all of the cells to lay out the faces in the output texture
    while ( currentPoint != pointsEnd )
    {
        std::cout << "Smoothing normals for point " << currentPoint.Index() << "/" << pointsEnd.Index() << "\r" << std::flush;

        VC_PointType p = currentPoint.Value();
        VC_PixelType currentNormal;
        inputMesh->GetPointData( currentPoint.Index(), &currentNormal );

        neighborCount = 0;
        neighborAvg[0] = 0;
        neighborAvg[1] = 0;
        neighborAvg[2] = 0;
        neighborPoint = inputMesh->GetPoints()->Begin(); 
        
        // find neighborhood for current point (p) within radius
        pointsLocator->FindPointsWithinRadius( p, smoothingFactor, neighborhood );
        neighborCount = neighborhood.size();

        for ( int i = 0; i < neighborCount; ++i ) {
            pointID = neighborhood[i];
            VC_PointType p2 = inputMesh->GetPoint( pointID );
            VC_PixelType neighborNormal;
            inputMesh->GetPointData( neighborPoint.Index(), &neighborNormal );

            neighborAvg[0] += neighborNormal[0];
            neighborAvg[1] += neighborNormal[1];
            neighborAvg[2] += neighborNormal[2];
        }
        if( neighborCount > 0) {
            // Calculate neighborhood's normal average and smooth
            currentNormal[0] = neighborAvg[0] / neighborCount;
            currentNormal[1] = neighborAvg[1] / neighborCount;
            currentNormal[2] = neighborAvg[2] / neighborCount;
            outputMesh->SetPointData( currentPoint.Index(), currentNormal );
        }

        ++currentPoint;
    }
    std::cout << std::endl;
        
    return outputMesh;
}

VC_MeshType::Pointer resample ( VC_MeshType::Pointer  inputMesh,
                                int                   NumberOfSamples,
                                float                 Gradation ) {

    // declare pointer to new Mesh object to be returned
    VC_MeshType::Pointer  outputMesh = VC_MeshType::New();

    // export mesh to stl, stl needed to import into ACVD's vtkSurface class    
    itk::STLMeshIOFactory::RegisterOneFactory();
    typedef itk::MeshFileWriter< VC_MeshType > WriterType;
    WriterType::Pointer writer = WriterType::New();
    writer->SetFileName( "temp.stl" );
    writer->SetInput( inputMesh );
    try
    {
        writer->Update();
    }
    catch( itk::ExceptionObject & excp )
    {
        std::cerr << excp << std::endl;
    }
    itk::STLMeshIO::Pointer meshIO = itk::STLMeshIO::New();
    meshIO->Print( std::cout );
   
    // ACVD's vtkSurface class used for resampling
    vtkSurface *Mesh = vtkSurface::New();
    Mesh->CreateFromFile( "temp.stl" );
    Mesh->GetCellData()->Initialize();
    Mesh->GetPointData()->Initialize();

    // initialize paramaters needed for ACVD resampling 
    int SubsamplingThreshold = 10;
    int QuadricsOptimizationLevel = 1;
    char outputfile[500];
    strcpy (outputfile, "simplification.ply");

    vtkIsotropicDiscreteRemeshing *Remesh = vtkIsotropicDiscreteRemeshing::New();

    Remesh->SetInput(Mesh);
    Remesh->SetFileLoadSaveOption(0);
    Remesh->SetNumberOfClusters(NumberOfSamples);
    Remesh->SetConsoleOutput(2);
    Remesh->SetSubsamplingThreshold(SubsamplingThreshold);
    Remesh->GetMetric()->SetGradation(Gradation);
    Remesh->Remesh();

    // code from ACVD.cxx example, optimization of resampled mesh
    if (QuadricsOptimizationLevel != 0) {
        // Note : this is an adaptation of Siggraph 2000 Paper :
        // Out-of-core simplification of large polygonal models
        vtkIntArray *Clustering = Remesh->GetClustering();

        char REALFILE[5000];
        char FileBeforeProcessing[500];
        strcpy (FileBeforeProcessing,"smooth_");
        strcat (FileBeforeProcessing, outputfile);
        strcpy (REALFILE, FileBeforeProcessing);
        Remesh->GetOutput()->WriteToFile(REALFILE);

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

    // save the output mesh to .ply format
    char REALFILE[500];
    strcpy (REALFILE, "");
    strcat (REALFILE, outputfile);

    Remesh->GetOutput()->WriteToFile(REALFILE);

    Remesh->Delete();
    Mesh->Delete();

    // Read in resampled mesh and return it
    // try to convert the ply to an ITK mesh
    int meshWidth = -1;
    int meshHeight = -1;
    if ( !volcart::io::ply2itkmesh("simplification.ply", outputMesh, meshWidth, meshHeight) ) {
        exit( -1 );
    };

    return outputMesh;
}
