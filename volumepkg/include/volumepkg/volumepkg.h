#pragma once

#include <cstdlib>
#include <iostream>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

#include <boost/filesystem.hpp>
#include "common/types/OrderedPointSet.h"
#include "common/types/Point.h"
#include "common/types/Texture.h"
#include "common/types/Volume.h"
#include "common/vc_defines.h"
#include "external/json.hpp"
#include "volumepkg/volumepkg_version.h"

/**
 * @class VolumePkg
 * This class exists to be a container for all of the data about a particular
 * set of data. It holds the slices, segmentations, mesh and texture data.
 */
class VolumePkg
{
public:
    /**
     * These are the Constructors, this first one is used to create a new volume
     * package and the second is for opening existing ones
     * @param file_location This is where you want to store the base directory
     * of the volume package
     * @param version This is the version of Volpkg you want to use, the current
     * version is 3 and is the only one that will work
     */
    VolumePkg(const boost::filesystem::path& file_location, int version);
    VolumePkg(
        const boost::filesystem::path& file_location); 

    /**
     * This function writes the Volpkg out to the disk when you create it
     * initially, it saves the metadata and builds the directory tree
     * @return An integer signalling success or failure
     */
    int initialize();

    // Accessors for volume
    /**
     * Returns the Volume information stored as a Volume type
     * @return VolumeType
     * @see common/types/Volume.h
     */
    const volcart::Volume& volume() const { return vol_; }

    volcart::Volume& volume() { return vol_; }
    // Debug
    /**
     * Prints the contents of the JSON file where the metadata is stored, mainly
     * used for Debug
     */
    void printJSON() const { config.printObject(); };

    /**
     * Prints the locations of the directories, mainly used for Debug
     */
    void printDirs() const
    {
        std::cout << "root: " << root_dir << " seg: " << segs_dir
                  << " slice: " << slice_dir << std::endl;
    };

    /**
     * Gets the name of the VolumePkg you are currently working on
     * @return Name of the Volume package
     */
    std::string getPkgName() const;

    /**
     * Gets the version that this VolumePkg is, current version is 3
     * @return integer that represents the version of VolumePkg
     */
    int getVersion() const;  // Changed type from double to int

    /**
     * Returns how many slices there are in this set of data
     * @return integer representing the number of slices
     */
    int getNumberOfSlices() const;

    /**
     * Returns the width of the slices, this is the same for all slices in a
     * VolumePkg
     * @return integer that represents the slice width
     */
    int getSliceWidth() const;

    /**
     * Returns the height of the slices in the data, this is the same for all
     * slices in a VolumePkg
     * @return Integer represents the height of the slices
     */
    int getSliceHeight() const;

    /**
     * Returns the size of the voxels in the data, this is the same for all
     * voxels in a VolumePkg
     * @return Double that represents the size of the voxels
     */
    double getVoxelSize() const;

    /**
     * Returns the thickness of the material that was scanned
     * @return Thickness of material scan
     */
    double getMaterialThickness() const;

    // Metadata Assignment
    /**
     * Checks to see if the VolumePkg is read only
     * @return Bool that states if the data is read only
     */
    bool readOnly() const { return _readOnly; };

    /**
     * Checks to see if the VolumePkg is read only and stores it in a variable
     * @param variable where the value of _readOnly is stored
     */
    void readOnly(bool b) { _readOnly = b; };

    // set a metadata key to a value
    /**
     * Sets a particular metadata value to a key so that it can be quickly found
     * later
     * @param key what the metadata is set to
     * @param value metadata that you want to store
     * @return Integer indicating success
     */
    template <typename T>
    int setMetadata(const std::string& key, T value)
    {
        if (_readOnly) {
            volcart::ERR_READONLY();
        }

        config.set<T>(key, value);
        return EXIT_SUCCESS;
    }

    // Metadata Export
    /**
     * Saves the metadata to a file
     * @param filePath File path where you want the metadata to be stored
     */
    void saveMetadata(const boost::filesystem::path& filePath);

    /**
     * Saves the metadata to a file determined by the program
     */
    void saveMetadata();

    // Slice manipulation
    /**
     * Allows you to set the slice height and width
     * @param index Slice number that you want to store data for
     * @param slice Slice that contains the information to set height and width
     */
    bool setSliceData(size_t index, const cv::Mat& slice);

    // Segmentation functions
    /**
     * Creates a new segmentation
     * @return name of the segmentation created
     */
    std::string newSegmentation();
    /**
     * Returns a list of the current segmentations for that VolumePkg
     * @return a vector of strings that contains the names of all the
     * segmentations for the VolumePkg
     */
    std::vector<std::string> getSegmentations() const;

    /**
     * Set the active segmentation to be a particular segmentation
     * @param name of the segmentation you want to be the active one
     */
    void setActiveSegmentation(const std::string&);
    /**
     * Get the name of the segmentation currently active
     * @return string containing the name of the active segmentation
     */
    std::string getActiveSegmentation();
    /**
     * Returns the file path of the segmentation that is currently active
     * @return file path where the active segmentation is
     */
    boost::filesystem::path getActiveSegPath();

    /**
     * This opens the file containing the information for the points that make
     * up the Volume
     * @return An OrderedPointSet which contains all of the points on the Volume
     * @see common/types/OrderedPointSet.h
     * @see common/types/PointSet.h
     */
    volcart::OrderedPointSet<volcart::Point3d> openCloud() const;

    /**
     * Gets the file path where the mesh for the currently active segmentation
     * is stored
     * @return Boost File path
     */
    boost::filesystem::path getMeshPath() const;

    /**
     * Returns the file that contains the Texture Data for the Volume
     * @return a Mat with the texture data
     */
    cv::Mat getTextureData() const;

    /**
     * Saves the points of the Volume that may have been altered due to
     * segmentation
     * @param segmentedCloud The set of points returned by the segmentation
     * algorithm that need to be saved
     * @return Integer indicating success
     */
    int saveCloud(
        const volcart::OrderedPointSet<volcart::Point3d>& segmentedCloud) const;

    /**
     * Generates a mesh from the Points provided and saves it to the
     * Segmentation folder of the active Segmentation
     * @param segmentedCloud The set of points returned by the segmentation
     * algorithm that need to be meshed and saved
     * @return an Integer indicating success
     */
    int saveMesh(
        const volcart::OrderedPointSet<volcart::Point3d>& segmentedCloud) const;

    /**
     * Saves a generated mesh along with the Texture information that is
     * provided
     * @param mesh Mesh that was generated from the points in the cloud of the
     * current segmentation
     * @param texture Texture information for a mesh
     * @see common/types/Texture.h
     */
    void saveMesh(
        const volcart::ITKMesh::Pointer mesh,
        const volcart::Texture& texture) const;

    /**
     * Saves the texture data for the current segmentation
     * @param texture Texture information as a Mat
     * @param name automatcially set to be textured and represents the name of
     * the file where this is stored
     */
    void saveTextureData(
        const cv::Mat& texture, const std::string& name = "textured");

    /**
     * Saves the texture data for the current segmentation
     * @param texture Texture information
     * @see common/types/Texture.h
     * @param index Tells the function which slice to use the texture data from,
     * automatically set to 0
     */
    void saveTextureData(volcart::Texture texture, int index = 0)
    {
        saveTextureData(texture.getImage(index));
    }

private:
    /**
     * Bool that tells if the Volume Package is read only
     */
    bool _readOnly = true;

    /**
     * Contains the Metadata for the Volume package
     * @see common/types/Metadata.h
     */
    volcart::Metadata config;

    /**
     * Contains the information stored in the Volume
     * @see common/types/Volume.h
     */
    volcart::Volume vol_;

    // Directory tree
    /**
     * Makes the subdirectories for the Volume Package
     * @return integer indicating success
     */
    int _makeDirTree();
    /**
     * The root directory of the Volume package, stores the other directories
     */
    boost::filesystem::path root_dir;
    /**
     * The directory containing the Segmentations that have been made
     */
    boost::filesystem::path segs_dir;
    /**
     * The directory containing the slices that the volume represents
     */
    boost::filesystem::path slice_dir;

    // Note to Seth: This function isn't implemented and Clion found no usages
    // of it?
    int getNumberOfSliceCharacters();
    /**
     * This is the segmentation that is currently being worked on
     */
    std::string activeSeg = "";
    /**
     * The list of all the segmentations for a specific VolumePkg
     */
    std::vector<std::string> segmentations;
    /**
     * Sets up which version of the Volume Package you're using and associates
     * all the keys with information from the dictonary
     * @param dict Which set of data types you want to use to create the
     * VolumePkg, corresponds to the version
     * @param version which version of the Volume Package you want to use,
     * current is 3
     * @return Inital metadata
     * @see common/types/Metadata.h
     */
    static volcart::Metadata _initConfig(
        const volcart::Dictionary& dict, int version);
};
