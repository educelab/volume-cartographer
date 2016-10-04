#pragma once

#include <cstdlib>
#include <iostream>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

#include <boost/filesystem.hpp>
#include "common/types/Texture.h"
#include "common/types/Volume.h"
#include "common/vc_defines.h"
#include "external/json.hpp"
#include "volumepkg/volumepkg_version.h"
#include "common/types/OrderedPointSet.h"
#include "common/types/Point.h"

class VolumePkg
{
public:
    // Constructors
    VolumePkg(const boost::filesystem::path& file_location, int version);

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
                  << " slice: " << slice_dir << std::endl;
    };

    // Metadata Retrieval
    std::string getPkgName() const;

    int getVersion() const;  // Changed type from double to int

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
    void saveMetadata(const boost::filesystem::path& filePath);

    void saveMetadata();

    // Slice manipulation
    bool setSliceData(size_t index, const cv::Mat& slice);

    // Segmentation functions
    std::string newSegmentation();
    std::vector<std::string> getSegmentations() const;

    void setActiveSegmentation(const std::string&);
    std::string getActiveSegmentation();
    boost::filesystem::path getActiveSegPath();

    volcart::OrderedPointSet<volcart::Point3d> openCloud() const;

    boost::filesystem::path getMeshPath() const;

    cv::Mat getTextureData() const;

    int saveCloud(
        const volcart::OrderedPointSet<volcart::Point3d> segmentedCloud) const;

    int saveMesh(
            volcart::OrderedPointSet<volcart::Point3d> segmentedCloud) const;

    void saveMesh(
        const VC_MeshType::Pointer& mesh, volcart::Texture& texture) const;

    void saveTextureData(
        const cv::Mat& texture, const std::string& name = "textured");

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

    int getNumberOfSliceCharacters();
    std::string activeSeg = "";
    std::vector<std::string> segmentations;

    static volcart::Metadata _initConfig(
        const volcart::Dictionary& dict, int version);
};
