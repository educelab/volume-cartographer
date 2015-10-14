#ifndef _VOLUMEPKG_H_
#define _VOLUMEPKG_H_

#include <stdlib.h>
#include <time.h>

#include <iostream>
#include <boost/filesystem.hpp>

// These boost libraries cause problems with QT4 + Boost 1.57. This is a workaround.
// https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
    #include <boost/foreach.hpp>
    #include <boost/lexical_cast.hpp>
#endif

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "volumepkgcfg.h"
#include "volumepkg_version.h"
#include "reslice.h"
#include "orderedPCDMesher.h"
#include "../texture/CMesh.h"
#include "../texture/CPlyHelper.h"

// Indices for indexing into vectors
#define VC_INDEX_X 0
#define VC_INDEX_Y 1
#define VC_INDEX_Z 2

class VolumePkg {
public:
    VolumePkg(std::string);

    // Debug
    void printObject();

    // Metadata Retrieval
    std::string getPkgName();
    double getVersion();
    int getNumberOfSlices();
    int getSliceWidth();
    int getSliceHeight();
    double getVoxelSize();
    double getMaterialThickness();

    // Metadata Assignment
    int setMetadata(std::string, int);
    int setMetadata(std::string, double);
    int setMetadata(std::string, std::string);

    // Metadata Export
    void saveMetadata();
    void saveMetadata(std::string filePath);

    // Data Retrieval
    cv::Mat getSliceData(int);
    std::string getSlicePath(int);
    std::string getNormalAtIndex(int);

    // Segmentation functions
    std::vector<std::string> getSegmentations();
    void setActiveSegmentation(std::string);
    std::string newSegmentation();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr openCloud();
    ChaoVis::CMesh openMesh();
    ChaoVis::CMesh openTexturedMesh();
    std::string getMeshPath();
    cv::Mat getTextureData();
    void saveCloud(pcl::PointCloud<pcl::PointXYZRGB>);
    void saveMesh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
    void saveTexturedMesh(ChaoVis::CMesh);
    void saveTextureData(cv::Mat, std::string = "texture");
    Reslice reslice(cv::Vec3f, cv::Vec3f, cv::Vec3f, uint32_t, uint32_t);

private:
    VolumePkgCfg config;
    std::string location;
    boost::filesystem::path segdir;
    int getNumberOfSliceCharacters();
    std::string activeSeg = "";
    std::vector<std::string> segmentations;

    std::string findKeyType(std::string);
    uint16_t interpolateAt(cv::Vec3f point);
};

#endif // _VOLUMEPKG_H_
