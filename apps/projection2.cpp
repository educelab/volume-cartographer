// projection.cpp
// Seth Parker 10/2015
// Project the mesh onto each slice in order to check the quality of segmentation
#include <stdio.h>
#include <map>

#include <opencv2/opencv.hpp>
#include <boost/format.hpp>

#include "vc_defines.h"
#include "vc_datatypes.h"
#include "shapes.h"

#include "io/ply2itk.h"
#include "itk2vtk.h"
#include "volumepkg.h"

#include <vtkPLYReader.h>
#include <vtkPLYWriter.h>
#include <vtkCutter.h>
#include <vtkStripper.h>
#include <vtkPlane.h>

#define RED cv::Scalar( 0, 0, 255 )

vtkSmartPointer<vtkPolyData> CreatePlane();

int main( int argc, char *argv[] ) {
  printf("Running tool: vc_projection\n");
  std::cout << std::endl;
  if (argc < 4) {
    printf("Usage: vc_projection volpkg seg-id output-dir\n");
    exit(-1);
  }

  // Load the volume package
  VolumePkg volpkg(argv[1]);
  volpkg.setActiveSegmentation(argv[2]);
  std::string outputDir = argv[3];
  int width = volpkg.getSliceWidth();
  int height = volpkg.getSliceHeight();

  // Get Mesh
  vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
  reader->SetFileName ( "0-decim.ply" );
  reader->Update();

  // Setup plane
  vtkSmartPointer<vtkPlane> cutPlane = vtkSmartPointer<vtkPlane>::New();
  cutPlane->SetOrigin( width/2, height/2, 0 );
  cutPlane->SetNormal(0,0,1);
  double z_min = std::floor(reader->GetOutput()->GetBounds()[4]);
  double z_max = std::ceil(reader->GetOutput()->GetBounds()[5]);

  // Setup cutting and stripping pipeline
  vtkSmartPointer<vtkCutter> cutter = vtkSmartPointer<vtkCutter>::New();
  cutter->SetCutFunction(cutPlane);
  cutter->SetInputConnection(reader->GetOutputPort());

  vtkSmartPointer<vtkStripper> stripper = vtkSmartPointer<vtkStripper>::New();
  stripper->SetInputConnection(cutter->GetOutputPort());

  // Iterate over every z-index in the range between z_min and z_max
  // Cut the mesh and draw its corresponding intersection onto a new output image
  cv::Mat outputImg;
  std::vector<cv::Point> contour;
  for( int it = z_min; it < z_max; ++it ) {
    cutPlane->SetOrigin( width/2, height/2, it );
    stripper->Update();
    vtkSmartPointer<vtkPolyData> intersection = stripper->GetOutput();

    outputImg = cv::Mat::zeros( height, width, CV_8UC1 );
    for ( vtkIdType c_id = 0; c_id < intersection->GetNumberOfCells(); ++c_id ) {
      vtkCell *inputCell = intersection->GetCell(c_id); // input cell
      for ( vtkIdType p_it = 0; p_it < inputCell->GetNumberOfPoints(); ++p_it ) {
        vtkIdType p_id = inputCell->GetPointId(p_it);
        contour.push_back(cv::Point(intersection->GetPoint(p_id)[0], intersection->GetPoint(p_id)[1]));
      }
      cv::polylines( outputImg, contour, false, 255, 1, CV_AA );
      contour.clear();
    }

    // Save the output to the provided directory
    std::string path = outputDir + "/" + std::to_string(it) + ".png";
    cv::imwrite(path, outputImg);

  }

  return EXIT_SUCCESS;
}
