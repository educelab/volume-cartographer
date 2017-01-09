#include <iostream>

#include <boost/filesystem.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "core/io/OBJWriter.h"
#include "core/types/Texture.h"
#include "core/types/VolumePkg.h"
#include "core/vc_defines.h"

using namespace volcart;
namespace fs = boost::filesystem;

VolumePkg* _volpkg;
cv::Mat _texture, _perPixelMask, _currentSlice;
volcart::PerPixelMap _perPixelMap;

ITKPoint lookup2Dto3D(int x, int y);
#define RED cv::Scalar(0, 0, 255)

int main(int /*argc*/, char* argv[])
{

    _volpkg = new VolumePkg(argv[1]);
    _volpkg->setActiveSegmentation(argv[2]);
    fs::path segPath =
        fs::canonical(fs::path(_volpkg->getMeshPath()).parent_path());

    // Load the initial stuff
    _texture = cv::imread(segPath.string() + "/textured.png", -1);
    std::cout << "texture: " << _texture.cols << ", " << _texture.rows
              << std::endl;

    _perPixelMask = cv::imread(
        segPath.string() + "/PerPixelMask.png", cv::IMREAD_GRAYSCALE);
    std::cout << "mask: " << _perPixelMask.cols << ", " << _perPixelMask.rows
              << std::endl;

    std::string ppm_path = segPath.string() + "/PerPixelMapping.yml.gz";
    _perPixelMap = PerPixelMap::ReadPPM(ppm_path);

    // ROI params
    int tl_x = std::stoi(argv[3]);
    int tl_y = std::stoi(argv[4]);
    int width = std::stoi(argv[5]);
    int height = std::stoi(argv[6]);

    cv::Point top_left(tl_x, tl_y);
    cv::Rect roi_rect = cv::Rect(top_left.x, top_left.y, width, height);
    cv::Mat maskROI = _perPixelMask(roi_rect);
    cv::Mat textureROI = _texture(roi_rect);

    // Feature location
    int feature_x = std::stoi(argv[7]);
    int feature_y = std::stoi(argv[8]);
    cv::Vec6d mapInfo = _perPixelMap(feature_y, feature_x);
    auto feature_slice_x = static_cast<int>(std::round(mapInfo(0)));
    auto feature_slice_y = static_cast<int>(std::round(mapInfo(1)));
    auto feature_slice_z = static_cast<int>(std::floor(mapInfo(2)));

    // Iterate over ROI
    ITKMesh::Pointer outputMesh = ITKMesh::New();
    volcart::Texture texture;
    texture.addImage(textureROI);
    double u, v;
    unsigned long point_counter = 0;
    std::vector<cv::Point2d> intersections;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (maskROI.at<unsigned char>(y, x) == 255) {
                mapInfo = _perPixelMap(top_left.y + y, top_left.x + x);
                ITKPoint pt;
                ITKPixel normal;
                pt[0] = mapInfo(0);
                pt[1] = mapInfo(1);
                pt[2] = mapInfo(2);
                if (std::floor(mapInfo(2)) == feature_slice_z) {
                    intersections.emplace_back(mapInfo(0), mapInfo(1));
                }
                normal[0] = mapInfo(3);
                normal[1] = mapInfo(4);
                normal[2] = mapInfo(5);
                u = static_cast<double>(x) / (width - 1);
                v = static_cast<double>(y) / (height - 1);
                texture.uvMap().set(point_counter, cv::Vec2d(u, v));
                outputMesh->SetPoint(point_counter, pt);
                outputMesh->SetPointData(point_counter, normal);
                ++point_counter;
            }
        }
    }

    // generate the faces
    ITKCell::CellAutoPointer cellpointer;
    unsigned long cell_counter = 0;
    for (int i = 1; i < height; ++i) {
        for (int j = 1; j < width; ++j) {
            int v1, v2, v3, v4;
            v1 = i * width + j;
            v2 = v1 - 1;
            v3 = v2 - width;
            v4 = v1 - width;

            cellpointer.TakeOwnership(new ITKTriangle);
            cellpointer->SetPointId(0, v1);
            cellpointer->SetPointId(1, v2);
            cellpointer->SetPointId(2, v3);
            outputMesh->SetCell(cell_counter, cellpointer);
            ++cell_counter;

            cellpointer.TakeOwnership(new ITKTriangle);
            cellpointer->SetPointId(0, v1);
            cellpointer->SetPointId(1, v3);
            cellpointer->SetPointId(2, v4);
            outputMesh->SetCell(cell_counter, cellpointer);
            ++cell_counter;
        }
    }

    ///// Write the ROI + Texture image /////
    cv::Mat texture8bpc = _texture.clone();
    cv::normalize(texture8bpc, texture8bpc, 0, 255, cv::NORM_MINMAX, CV_8UC3);
    cv::cvtColor(texture8bpc, texture8bpc, cv::COLOR_GRAY2BGR);

    // Draw the ROI box
    cv::rectangle(texture8bpc, roi_rect, RED, 3);
    // Draw the feature dot
    cv::circle(texture8bpc, cv::Point(feature_x, feature_y), 15, RED, 2);
    cv::circle(texture8bpc, cv::Point(feature_x, feature_y), 3, RED, -1);
    // Write the image
    cv::imwrite("ROI_texture.png", texture8bpc);

    ///// Write the slice intersection image /////
    cv::Mat slice_view = _volpkg->volume().getSliceDataCopy(feature_slice_z);
    cv::normalize(slice_view, slice_view, 0, 255, cv::NORM_MINMAX, CV_8UC3);
    cv::cvtColor(slice_view, slice_view, cv::COLOR_GRAY2BGR);

    // Draw the intersections
    for (auto i : intersections)
        cv::circle(slice_view, i, 1, RED, -1);

    // Draw the feature
    cv::circle(
        slice_view, cv::Point(feature_slice_x, feature_slice_y), 15, RED, 2);
    cv::circle(
        slice_view, cv::Point(feature_slice_x, feature_slice_y), 3, RED, -1);
    std::string name =
        "ROI_intersection_" + std::to_string(feature_slice_z) + ".png";
    cv::imwrite(name, slice_view);

    ///// Write the mesh /////
    volcart::io::OBJWriter writer("ROI.obj", outputMesh);
    cv::Mat mesh_texture = texture8bpc(roi_rect);
    writer.setTexture(mesh_texture);
    writer.setUVMap(texture.uvMap());
    writer.write();

    return EXIT_SUCCESS;
}

ITKPoint lookup2Dto3D(int y, int x)
{
    ITKPoint pt;
    pt[0] = _perPixelMap(y, x)(0);
    pt[1] = _perPixelMap(y, x)(1);
    pt[2] = _perPixelMap(y, x)(2);
    return pt;
}
