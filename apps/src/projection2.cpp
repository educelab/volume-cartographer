// projection.cpp
// Seth Parker 10/2015
// Project the mesh onto each slice in order to check the quality of
// segmentation
#include <map>
#include <stdio.h>

#include <boost/format.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <vtkCell.h>
#include <vtkCutter.h>
#include <vtkPLYReader.h>
#include <vtkPlane.h>
#include <vtkSmartPointer.h>
#include <vtkStripper.h>
#include "core/types/VolumePkg.h"
#include "core/vc_defines.h"

int main(int argc, char* argv[])
{
    printf("Running tool: vc_projection\n");
    if (argc < 4) {
        std::cout << std::endl;
        printf("Usage: vc_projection volpkg seg-id output-dir\n");
        exit(-1);
    }

    // Load the volume package
    VolumePkg volpkg(argv[1]);
    volpkg.setActiveSegmentation(argv[2]);
    std::string outputDir = argv[3];
    int width = volpkg.getSliceWidth();
    int height = volpkg.getSliceHeight();
    int padding = std::to_string(volpkg.getNumberOfSlices()).size();

    // Get Mesh
    vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
    reader->SetFileName("0-decim.ply");
    reader->Update();

    // Setup plane
    vtkSmartPointer<vtkPlane> cutPlane = vtkSmartPointer<vtkPlane>::New();
    cutPlane->SetOrigin(width / 2, height / 2, 0);
    cutPlane->SetNormal(0, 0, 1);
    auto z_min =
        static_cast<int>(std::floor(reader->GetOutput()->GetBounds()[4]));
    auto z_max =
        static_cast<int>(std::ceil(reader->GetOutput()->GetBounds()[5]));

    // Bounds checks
    if (z_min < 0)
        z_min = 0;
    if (z_max >= volpkg.getNumberOfSlices())
        z_max = volpkg.getNumberOfSlices() - 1;
    if (z_min == z_max)
        z_max += 1;

    // Setup cutting and stripping pipeline
    vtkSmartPointer<vtkCutter> cutter = vtkSmartPointer<vtkCutter>::New();
    cutter->SetCutFunction(cutPlane);
    cutter->SetInputConnection(reader->GetOutputPort());

    vtkSmartPointer<vtkStripper> stripper = vtkSmartPointer<vtkStripper>::New();
    stripper->SetInputConnection(cutter->GetOutputPort());

    // Iterate over every z-index in the range between z_min and z_max
    // Cut the mesh and draw its corresponding intersection onto a new output
    // image
    cv::Mat outputImg;
    std::vector<cv::Point> contour;
    for (int it = z_min; it < z_max; ++it) {
        std::cerr << "volcart::projection::Projecting " << std::to_string(it)
                  << "\r" << std::flush;
        cutPlane->SetOrigin(width / 2, height / 2, it);
        stripper->Update();
        vtkSmartPointer<vtkPolyData> intersection = stripper->GetOutput();

        outputImg = cv::Mat::zeros(height, width, CV_8UC1);
        for (vtkIdType c_id = 0; c_id < intersection->GetNumberOfCells();
             ++c_id) {
            vtkCell* inputCell = intersection->GetCell(c_id);  // input cell
            for (vtkIdType p_it = 0; p_it < inputCell->GetNumberOfPoints();
                 ++p_it) {
                vtkIdType p_id = inputCell->GetPointId(p_it);
                contour.push_back(cv::Point(
                    static_cast<int>(intersection->GetPoint(p_id)[0]),
                    static_cast<int>(intersection->GetPoint(p_id)[1])));
            }
            cv::polylines(outputImg, contour, false, 255, 1, cv::LINE_AA);
            contour.clear();
        }

        // Save the output to the provided directory
        std::stringstream filename;
        filename << std::setw(padding) << std::setfill('0') << it << ".png";
        std::string path = outputDir + "/" + filename.str();
        cv::imwrite(path, outputImg);
    }
    std::cerr << std::endl;

    return EXIT_SUCCESS;
}
