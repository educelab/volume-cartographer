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

#include "common/io/PLYReader.h"
#include "common/vc_defines.h"
#include "volumepkg/volumepkg.h"

#define RED cv::Scalar(0, 0, 255)

using namespace volcart;

int main(int argc, char* argv[])
{
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

    // Load the mesh
    auto mesh = ITKMesh::New();
    volcart::io::PLYReader(volpkg.getMeshPath(), mesh);

    // PNG Compression params
    std::vector<int> compression_params;
    compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);

    // Sort the points by z-index
    std::map<int, std::vector<int>> z_map;

    for (auto point = mesh->GetPoints()->Begin();
         point != mesh->GetPoints()->End(); ++point) {

        // skip null points
        if (point->Value()[INDEX_Z] == -1)
            continue;

        // get the z-index for this point (floored int)
        int this_z = (int)point->Value()[INDEX_Z];

        // add point id to the vector for this z-index
        // make a vector for this z-index if it doesn't exist in the map
        auto lookup_z = z_map.find(this_z);
        if (lookup_z != z_map.end()) {
            lookup_z->second.push_back(point->Index());
        } else {
            std::vector<int> z_vector;
            z_vector.push_back(point->Index());
            z_map.insert({this_z, z_vector});
        }
    }

    // Iterate over each z-index and generate a projected slice image
    for (auto z_id = z_map.begin(); z_id != z_map.end(); ++z_id) {
        std::cout << "Projecting slice " + std::to_string(z_id->first) + "\r"
                  << std::flush;
        // get the slice image and cvt to CV_8UC3
        // .clone() to make sure we don't modify the cached version
        cv::Mat slice = volpkg.volume().getSliceData(z_id->first).clone();
        slice.convertTo(slice, CV_8U, 255.0 / 65535.0);
        cv::cvtColor(slice, slice, CV_GRAY2BGR);

        // Iterate over the points for this z-index and project the points
        for (auto p_id = z_id->second.begin(); p_id != z_id->second.end();
             ++p_id) {

            ITKPoint point = mesh->GetPoint(*p_id);
            cv::Point pos;
            pos.x = cvRound(point[INDEX_X]);
            pos.y = cvRound(point[INDEX_Y]);

            cv::circle(slice, pos, 1, RED, -1);
        }

        // Generate an output path from the z-index
        std::stringstream outputPath;
        outputPath << boost::format(outputDir + "/proj%04i.png") % z_id->first;
        cv::imwrite(outputPath.str(), slice, compression_params);
    }
    std::cout << std::endl;

    return EXIT_SUCCESS;
}
