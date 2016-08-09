#ifndef _VOLUMEPKG_H_
#define _VOLUMEPKG_H_

#include <boost/filesystem.hpp>
#include <cstdlib>
#include <iostream>

// These boost libraries cause problems with QT4 + Boost 1.57. This is a
// workaround.
// https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#endif

#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include "external/json.hpp"
#include "common/vc_defines.h"
#include "volumepkg/volumepkg_version.h"
#include "common/types/Texture.h"
#include "common/types/Volume.h"

class VolumePkg
{
public:
    // Constructors
    VolumePkg(const boost::filesystem::path& file_location,
              double version);  // New volpkg, V.[version]

    VolumePkg(
        const boost::filesystem::path& file_location);  // Existing VolPkgs

    // Write to disk for the first time
    int initialize();

    // Accessor for volume
    const volcart::Volume& volume() const { return vol_; }

    volcart::Volume& volume() { return vol_; }
    // Debug
    void printJSON() const { config.printObject(); };

    void printDirs() const
    {
        std::cout << "root: " << root_dir << " seg: " << segs_dir
                  << " slice: " << slice_dir << " norm: " << norm_dir
                  << std::endl;
    };

    // Metadata Retrieval
    std::string getPkgName() const;

    double getVersion() const;

    int getNumberOfSlices() const;

    int getSliceWidth() const;

    int getSliceHeight() const;

    double getVoxelSize() const;

    double getMaterialThickness() const;

    // Metadata Assignment
    bool readOnly() const { return _readOnly; };

    void readOnly(bool b) { _readOnly = b; };

    // set a metadata key to a value
    template <typename T>
    int setMetadata(const std::string& key, T value)
    {
        if (_readOnly) {
            VC_ERR_READONLY();
        }

        config.set<T>(key, value);
        return EXIT_SUCCESS;
    }

    // Metadata Export
    void saveMetadata(const std::string& filePath);

    void saveMetadata();

    // Slice manipulation
    bool setSliceData(size_t index, const cv::Mat& slice);

    // Segmentation functions
    std::string newSegmentation();
    std::vector<std::string> getSegmentations() const;

    void setActiveSegmentation(const std::string&);
    std::string getActiveSegmentation();
    boost::filesystem::path getActiveSegPath();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr openCloud() const;

    std::string getMeshPath() const;

    cv::Mat getTextureData() const;

    int saveCloud(
        const pcl::PointCloud<pcl::PointXYZRGB>& segmentedCloud) const;

    int saveMesh(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& segmentedCloud) const;

    void saveMesh(const VC_MeshType::Pointer& mesh,
                  volcart::Texture& texture) const;

    void saveTextureData(const cv::Mat& texture,
                         const std::string& name = "textured");

    void saveTextureData(volcart::Texture texture, int index = 0)
    {
        saveTextureData(texture.getImage(index));
    }

private:
    bool _readOnly = true;

    volcart::Metadata config;

    volcart::Volume vol_;

    // Directory tree
    int _makeDirTree();
    boost::filesystem::path root_dir;
    boost::filesystem::path segs_dir;
    boost::filesystem::path slice_dir;
    boost::filesystem::path norm_dir;

    int getNumberOfSliceCharacters();
    std::string activeSeg = "";
    std::vector<std::string> segmentations;

    static volcart::Metadata _initConfig(
        const volcart::Dictionary& dict,
        double version);
};

#endif  // _VOLUMEPKG_H_
