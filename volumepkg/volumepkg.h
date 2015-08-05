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
#include "picojson.h"

#include "volumepkgcfg.h"
#include "volumepkg_version.h"
#include "orderedPCDMesher.h"
#include "../texture/CMesh.h"
#include "../texture/CPlyHelper.h"

class VolumePkg {
public:
    // Constructors
    VolumePkg(std::string file_location, double version); // New volpkg, V.[version]
    VolumePkg(std::string file_location); // Existing VolPkgs

    // Write to Disk
    int initialize();

    // Debug
    void printJSON() { config.printObject(); };
    void printDirs() { std::cout << "root: " << root_dir << " seg: " << segs_dir << " slice: " << slice_dir << " norm: " << norm_dir << std::endl; };

    // Metadata Retrieval
    std::string getPkgName();
    double getVersion();
    int getNumberOfSlices();
    int getSliceWidth();
    int getSliceHeight();
    double getVoxelSize();
    double getMaterialThickness();

    // Metadata Assignment
    bool readOnly()         { return _readOnly; };
    void readOnly(bool b)   { _readOnly = b; };

    // set a metadata key to a value
    // Sorry for this templated mess. - SP 072015
    template<typename T>
    int setMetadata(std::string key, T value) {
        if (_readOnly) VC_ERR_READONLY();

        std::string keyType = findKeyType(key);
        if (keyType == "string") {
            try {
                std::string castValue = boost::lexical_cast<std::string>(value);
                config.setValue(key, castValue);
                return EXIT_SUCCESS;
            }
            catch(const boost::bad_lexical_cast &) {
                std::cerr << "ERROR: Given value \"" << value << "\" cannot be cast to type specified by dictionary (" << keyType << ")" << std::endl;
                return EXIT_FAILURE;
            }
        }
        else if (keyType == "int") {
            try {
                int castValue = boost::lexical_cast<int>(value);
                config.setValue(key, castValue);
                return EXIT_SUCCESS;
            }
            catch(const boost::bad_lexical_cast &) {
                std::cerr << "ERROR: Given value \"" << value << "\" cannot be cast to type specified by dictionary (" << keyType << ")" << std::endl;
                return EXIT_FAILURE;
            }
        }
        else if (keyType == "double") {
            try {
                double castValue = boost::lexical_cast<double>(value);
                config.setValue(key, castValue);
                return EXIT_SUCCESS;
            }
            catch(const boost::bad_lexical_cast &) {
                std::cerr << "ERROR: Given value \"" << value << "\" cannot be cast to type specified by dictionary (" << keyType << ")" << std::endl;
                return EXIT_FAILURE;
            }
        }
        else if (keyType == "") {
            return EXIT_FAILURE;
        }
        else {
            std::cerr << "ERROR: Value \"" << value << "\" not of type specified by dictionary (" << keyType << ")" << std::endl;
            return EXIT_FAILURE;
        }
    };

    // Metadata Export
    void saveMetadata(std::string filePath);
    void saveMetadata();

    // Data Retrieval
    cv::Mat getSliceData(int);
    std::string getSlicePath(int);
    std::string getNormalAtIndex(int);

    // Data Assignment
    int setSliceData(unsigned long index, cv::Mat slice);

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
    
private:
    bool _readOnly = true;

    VolumePkgCfg config;

    // Directory tree
    int _build();
    boost::filesystem::path root_dir;
    boost::filesystem::path segs_dir;
    boost::filesystem::path slice_dir;
    boost::filesystem::path norm_dir;

    int getNumberOfSliceCharacters();
    std::string activeSeg = "";
    std::vector<std::string> segmentations;

    std::string findKeyType(std::string);
};

#endif // _VOLUMEPKG_H_
