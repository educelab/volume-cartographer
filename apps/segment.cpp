#include <iostream>

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/console/parse.h>
#include <pcl/filters/conditional_removal.h>

#include "volumepkg.h"
#include "localResliceParticleSim/localResliceParticleSim.h"

// File paths
std::string segID = "";
std::string volpkgLocation = "";
std::string outputName = "";

// Options
int threshold = 1;
double startIndex = -1.0;
int stopOffset = -1;

int main(int argc, char *argv[]) {
    std::cout << "vc_segment" << std::endl;
    if (argc < 5) {
        std::cerr << "Usage:" << std::endl;
        std::cerr << argv[0] <<
        " --startIndex [Z-Index #] --stopOffset [value]} --seg [Seg ID #] --volpkg [volpkgpath]" <<
        std::endl;
        exit(EXIT_FAILURE);
    }

    // Option parsing

    // NOTE: Distance thresholding causes problems for resumable segmentations
    //       because you currently need to know the previous step distance.
    //       Removing until the pipeline is stronger.
    // pcl::console::parse_argument (argc, argv, "--threshold", threshold);
    // if (threshold == -1) {
    //   std::cout << "No Distance Threshold value given, defaulting to 1" << std::endl;
    //   threshold = 1;
    // }

    pcl::console::parse_argument(argc, argv, "--startIndex", startIndex);

    pcl::console::parse_argument(argc, argv, "--stopOffset", stopOffset);

    pcl::console::parse_argument(argc, argv, "--seg", segID);

    pcl::console::parse_argument(argc, argv, "--volpkg", volpkgLocation);
    if (volpkgLocation == "") {
        std::cerr << "ERROR: Incorrect/missing volpkg location!" << std::endl;
        exit(EXIT_FAILURE);
    }

    // Load volume package
    VolumePkg volpkg(volpkgLocation);
    // To-Do: Check to make sure the Seg ID is actually in the volpkg
    volpkg.setActiveSegmentation(segID);
    if (segID == "") {
        std::cerr << "ERROR: Incorrect/missing segmentation ID!" << std::endl;
        exit(EXIT_FAILURE);
    }
    if (volpkg.getVersion() < 2.0) {
        std::cerr << "ERROR: Volume package is version " << volpkg.getVersion() <<
        " but this program requires a version >= 2.0." << std::endl;
        exit(EXIT_FAILURE);
    }

    // Setup
    // Load the activeSegmentation's current cloud
    auto masterCloud = volpkg.openCloud();

    // Get some info about the cloud, including chain length and z-index's represented by seg.
    int chainLength = masterCloud->width;
    int iterations = masterCloud->height;
    pcl::PointXYZRGB min_p, max_p;
    pcl::getMinMax3D(*masterCloud, min_p, max_p);
    int minIndex = floor(masterCloud->points[0].z);
    int maxIndex = floor(max_p.z);

    // Setup the temp clouds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr immutableCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segPath(new pcl::PointCloud<pcl::PointXYZRGB>);

    // If no start index is given, our starting path is all of the points already on the largest slice index
    if (startIndex == -1.0) {
        startIndex = maxIndex;
        std::cout << "No starting index given, defaulting to Highest-Z: " << startIndex << std::endl;
    }


    // Prepare our clouds
    // Get the starting path pts.
    // Find our starting row. NOTE: This currently assumes segmentation distance threshold has always been 1
    int pathRow = startIndex - minIndex;
    for (int i = 0; i < chainLength; ++i) {
        pcl::PointXYZRGB temp_pt;
        temp_pt = masterCloud->points[i + (pathRow * chainLength)];
        if (temp_pt.z != -1) {
            segPath->push_back(temp_pt);
        }
    }

    // Starting paths must have the same number of points as the input width to maintain ordering
    if (segPath->width != chainLength) {
        std::cerr << std::endl;
        std::cerr << "ERROR: Starting chain length does not match expected chain length." << std::endl;
        std::cerr << "           Expected: " << chainLength << std::endl;
        std::cerr << "           Actual: " << segPath->width << std::endl;
        std::cerr << "       Consider using a lower starting index value." << std::endl << std::endl;
        exit(EXIT_FAILURE);
    }

    // Get the immutable points, i.e all pts before the starting path row
    for (int i = 0; i < (pathRow * chainLength); ++i) {
        pcl::PointXYZRGB temp_pt;
        temp_pt = masterCloud->points[i];
        immutableCloud->push_back(temp_pt);
    }
    immutableCloud->width = chainLength;
    immutableCloud->height = immutableCloud->points.size() / chainLength;
    immutableCloud->points.resize(immutableCloud->width * immutableCloud->height);

    // Run segmentation using path as our starting points
    pcl::PointCloud<pcl::PointXYZRGB> mutableCloud;
    volcart::segmentation::LocalResliceSegmentation segmentation(volpkg);
    mutableCloud = segmentation.segmentLayer(0.05);

    // Update the master cloud with the points we saved and concat the new points into the space
    *masterCloud = *immutableCloud;
    *masterCloud += mutableCloud;

    // Restore ordering information
    masterCloud->width = chainLength;
    masterCloud->height = masterCloud->points.size() / masterCloud->width;
    masterCloud->points.resize(masterCloud->width * masterCloud->height);

    // Save point cloud and mesh
    volpkg.saveCloud(*masterCloud);
    volpkg.saveMesh(masterCloud);

    exit(EXIT_SUCCESS);
}
