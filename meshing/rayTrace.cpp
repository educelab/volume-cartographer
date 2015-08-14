//
// Created by Media Team on 8/12/15.
//

#include "rayTrace.h"

// Parameters used to create cylindrical ray tracing
#define PI_X2 (2 * 3.1415926)
#define OUT_X 2000
#define D_THETA (PI_X2 / OUT_X)

namespace volcart {
  namespace meshing {
  	// returns a vector of vectors that holds the points of intersections and the corresponding normals
  	std::vector< std::vector<cv::Vec3f> > rayTrace(VC_MeshType::Pointer itkMesh, int aTraceDir, int &width, int &height) {

  	  // Convert the itk mesh to a vtk mesh
  	  vtkPolyData *vtkMesh = vtkPolyData::New();
      volcart::meshing::itk2vtk(itkMesh, vtkMesh);

  	  // Get the bounds of the mesh
      double bounds[6];
  	  vtkMesh->GetBounds(bounds);

      // Set ray width and height, used for texturing
      height = (int)(bounds[5] - bounds[4]);
      width = OUT_X;

  	  // Generate normals for the cells of the mesh
  	  vtkSmartPointer<vtkPolyDataNormals> calcNormals = vtkSmartPointer<vtkPolyDataNormals>::New();
  	  calcNormals->SetInputData(vtkMesh);
  	  calcNormals->ComputeCellNormalsOn();
  	  calcNormals->Update();
  	  vtkMesh = calcNormals->GetOutput();

  	  // Creat vtk OBBTree
  	  vtkSmartPointer<vtkOBBTree> obbTree = vtkSmartPointer<vtkOBBTree>::New();
      obbTree->SetDataSet(vtkMesh);
      obbTree->BuildLocator();

  	  vtkSmartPointer<vtkPoints> intersectPoints = vtkSmartPointer<vtkPoints>::New();
  	  vtkSmartPointer<vtkIdList> intersectCells = vtkSmartPointer<vtkIdList>::New();

  	  // Calculate the origin by averaging the bounds of each coordinate
  	  cv::Vec3f origin;
  	  origin(0) = (bounds[0] + bounds[1]) / 2;
  	  origin(1) = (bounds[2] + bounds[3]) / 2;

  	  // Essential data structure to return points and normals
  	  std::vector< std::vector<cv::Vec3f> > intersections;

  	  // For each slice/row generate rays and interpolate new points
  	  for (int z = (int)bounds[4]; z < (int)bounds[5]; ++z) {
      	int counter = 0;

      	int ycount = 0;
      	double radian = 0;
        
        std::cout << "\rRay tracing for Z index: " << z << "/" << (int)bounds[5] - 1 << std::flush;

      	// generate rays cylindrically
      	for (double r = 0; r < PI_X2; r += D_THETA, ycount++) {
      	  // Calculate the ray according to ray tracing direction
      	  if (aTraceDir == 1) {
          	// counterclockwise
          	radian -= D_THETA;
      	  } else {
        		// clockwise (default)
        		radian += D_THETA;
      	  }
      
      	  // The z component of the origin changes every iteration
      	  origin(2) = z;

      	  // Calculate direction of ray according to current degree of rotation along the cylinder
      	  cv::Vec3f direction(cos(radian), sin(radian), 0);
      	  cv::normalize(direction);

      	  // Create a second point along the ray using the origin and direction
      	  cv::Vec3f end_point = origin + 400*direction;
      	  //origin = origin - 1000*direction;

      	  double start[3] = {origin[0], origin[1], origin[2]};
      	  double end[3] = {end_point[0], end_point[1], end_point[2]};

      	  obbTree->IntersectWithLine(start, end, intersectPoints, intersectCells);

      	  if ( intersectPoints->GetNumberOfPoints() > 0 ) {
            std::vector<cv::Vec3f> textureInfo;
      	  	cv::Vec3f pt_pos;
        		cv::Vec3f pt_norm;
            cv::Vec3f uv_coord;

        		pt_pos[0] = intersectPoints->GetPoint(0)[0];
        		pt_pos[1] = intersectPoints->GetPoint(0)[1];
        		pt_pos[2] = intersectPoints->GetPoint(0)[2];
        		pt_norm[0] = vtkMesh->GetCellData()->GetNormals()->GetTuple(intersectCells->GetId(0))[0];
        		pt_norm[1] = vtkMesh->GetCellData()->GetNormals()->GetTuple(intersectCells->GetId(0))[1];
	        	pt_norm[2] = vtkMesh->GetCellData()->GetNormals()->GetTuple(intersectCells->GetId(0))[2];
            // REVISIT - unconventional way to store uv coordinates
            uv_coord[0] = ycount; // u coordinate
            uv_coord[1] = z - bounds[4];
            uv_coord[2] = 0;

	        	textureInfo.push_back(pt_pos);
	        	textureInfo.push_back(pt_norm);
            textureInfo.push_back(uv_coord);
	        	intersections.push_back(textureInfo);
      	  }
      	}
      }
      std::cout << std::endl;
      return intersections;
  	} // rayTrace

  } // namespace meshing
} // namespace volcart