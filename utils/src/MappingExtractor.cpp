#include <iostream>

#include <boost/filesystem.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "vc/core/io/OBJWriter.hpp"
#include "vc/core/types/PerPixelMap.hpp"
#include "vc/core/types/Texture.hpp"
#include "vc/core/types/VolumePkg.hpp"
#include "vc/core/vc_defines.hpp"

using namespace volcart;
namespace fs = boost::filesystem;

ITKPoint Lookup2Dto3D(int y, int x, volcart::PerPixelMap& map);

#define RED cv::Scalar(0, 0, 255)

int main(int /*argc*/, char** argv)
{
    VolumePkg* volpkg;
    cv::Mat perPixelMask, currentSlice;
    PerPixelMap perPixelMap;

    volpkg = new VolumePkg(argv[1]);
    volpkg->setActiveSegmentation(argv[2]);
    fs::path segPath =
        fs::canonical(fs::path(volpkg->getMeshPath()).parent_path());

    // Load the initial stuff
    auto tex = cv::imread(segPath.string() + "/textured.png", -1);
    std::cout << "texture: " << tex.cols << ", " << tex.rows << std::endl;

    perPixelMask = cv::imread(
        segPath.string() + "/PerPixelMask.png", cv::IMREAD_GRAYSCALE);
    std::cout << "mask: " << perPixelMask.cols << ", " << perPixelMask.rows
              << std::endl;

    perPixelMap = PerPixelMap::ReadPPM(segPath / "PerPixelMapping.ppm");

    // ROI params
    int tlX = std::stoi(argv[3]);
    int tlY = std::stoi(argv[4]);
    int width = std::stoi(argv[5]);
    int height = std::stoi(argv[6]);

    cv::Point topLeft(tlX, tlY);
    cv::Rect roiRect = cv::Rect(topLeft.x, topLeft.y, width, height);
    cv::Mat maskROI = perPixelMask(roiRect);
    cv::Mat textureROI = tex(roiRect);

    // Feature location
    int xfeature = std::stoi(argv[7]);
    int yfeature = std::stoi(argv[8]);
    cv::Vec6d mapInfo = perPixelMap(yfeature, xfeature);
    auto xFeatureSlice = std::lround(mapInfo(0));
    auto yFeatureSlice = std::lround(mapInfo(1));
    auto zFeatureSlice = static_cast<int>(std::floor(mapInfo(2)));

    // Iterate over ROI
    ITKMesh::Pointer outputMesh = ITKMesh::New();
    volcart::Texture texture;
    texture.addImage(textureROI);
    double u, v;
    uint64_t pointCounter = 0;
    std::vector<cv::Point2d> intersections;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (maskROI.at<uint8_t>(y, x) == 255) {
                mapInfo = perPixelMap(
                    static_cast<size_t>(topLeft.y) + y,
                    static_cast<size_t>(topLeft.x) + x);
                ITKPoint pt;
                ITKPixel normal;
                pt[0] = mapInfo(0);
                pt[1] = mapInfo(1);
                pt[2] = mapInfo(2);
                if (std::floor(mapInfo(2)) == zFeatureSlice) {
                    intersections.emplace_back(mapInfo(0), mapInfo(1));
                }
                normal[0] = mapInfo(3);
                normal[1] = mapInfo(4);
                normal[2] = mapInfo(5);
                u = static_cast<double>(x) / (width - 1);
                v = static_cast<double>(y) / (height - 1);
                texture.uvMap().set(pointCounter, cv::Vec2d(u, v));
                outputMesh->SetPoint(pointCounter, pt);
                outputMesh->SetPointData(pointCounter, normal);
                ++pointCounter;
            }
        }
    }

    // generate the faces
    ITKCell::CellAutoPointer cellpointer;
    uint64_t cellCounter = 0;
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
            outputMesh->SetCell(cellCounter, cellpointer);
            ++cellCounter;

            cellpointer.TakeOwnership(new ITKTriangle);
            cellpointer->SetPointId(0, v1);
            cellpointer->SetPointId(1, v3);
            cellpointer->SetPointId(2, v4);
            outputMesh->SetCell(cellCounter, cellpointer);
            ++cellCounter;
        }
    }

    ///// Write the ROI + Texture image /////
    cv::Mat texture8bpc = tex.clone();
    cv::normalize(texture8bpc, texture8bpc, 0, 255, cv::NORM_MINMAX, CV_8UC3);
    cv::cvtColor(texture8bpc, texture8bpc, cv::COLOR_GRAY2BGR);

    // Draw the ROI box
    cv::rectangle(texture8bpc, roiRect, RED, 3);
    // Draw the feature dot
    cv::circle(texture8bpc, cv::Point(xfeature, yfeature), 15, RED, 2);
    cv::circle(texture8bpc, cv::Point(xfeature, yfeature), 3, RED, -1);
    // Write the image
    cv::imwrite("ROI_texture.png", texture8bpc);

    ///// Write the slice intersection image /////
    cv::Mat sliceView = volpkg->volume()->getSliceDataCopy(zFeatureSlice);
    cv::normalize(sliceView, sliceView, 0, 255, cv::NORM_MINMAX, CV_8UC3);
    cv::cvtColor(sliceView, sliceView, cv::COLOR_GRAY2BGR);

    // Draw the intersections
    for (const auto& i : intersections) {
        cv::circle(sliceView, i, 1, RED, -1);
    }

    // Draw the feature
    cv::circle(sliceView, cv::Point(xFeatureSlice, yFeatureSlice), 15, RED, 2);
    cv::circle(sliceView, cv::Point(xFeatureSlice, yFeatureSlice), 3, RED, -1);
    std::string name =
        "ROI_intersection_" + std::to_string(zFeatureSlice) + ".png";
    cv::imwrite(name, sliceView);

    ///// Write the mesh /////
    volcart::io::OBJWriter writer("ROI.obj", outputMesh);
    cv::Mat meshTexture = texture8bpc(roiRect);
    writer.setTexture(meshTexture);
    writer.setUVMap(texture.uvMap());
    writer.write();

    return EXIT_SUCCESS;
}

ITKPoint Lookup2Dto3D(int y, int x, volcart::PerPixelMap& map)
{
    ITKPoint pt;
    pt[0] = map(y, x)(0);
    pt[1] = map(y, x)(1);
    pt[2] = map(y, x)(2);
    return pt;
}
