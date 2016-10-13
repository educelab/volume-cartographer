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
 * @brief The interface to the VolumePkg (.volpkg) file format.
 *
 * Provides access to volume, segmentation, and rendering data stored on disk.
 *
 * @ingroup VolumePackage
 *
 * @see apps/src/packager.cpp
 *      apps/src/metadata.cpp
 *      examples/src/volpkg.cpp
 *      examples/src/ResliceAnalysis.cpp
 */
class VolumePkg
{
public:
    /**
     * @brief Construct an empty VolumePkg of a specific version number.
     *
     * This will construct an empty VolumePkg in memory and set its expected
     * location on disk. Note: You must call initialize() before the file can
     * be written to and accessed. Only metadata keys may be modified before
     * initialize is called.
     * @param file_location The location to store the VolPkg
     * @param version Version of VolumePkg you wish to construct
     */
    VolumePkg(const boost::filesystem::path& file_location, int version);

    /**
     * @brief Construct a VolumePkg from a .volpkg file stored at file_location.
     * @param file_location The root of the VolumePkg file
     */
    VolumePkg(const boost::filesystem::path& file_location);

    /**
     * @brief Initialize an empty .volpkg file on disk.
     *
     * Used when setting up a new VolumePkg file. Returns `EXIT_FAILURE`
     * if VolumePkg is set to read-only or if file is unwritable.
     * @return `EXIT_SUCCESS` or `EXIT_FAILURE`
     */
    int initialize();

    /**
     * @brief Prints the JSON object that stores VolumePkg metadata. Debug only.
     */
    void printJSON() const { config.printObject(); };

    /**
     * @brief Prints the paths to important VolumePkg subdirectories.
     * Debug only.
     */
    void printDirs() const
    {
        std::cout << "root: " << root_dir << " seg: " << segs_dir
                  << " slice: " << slice_dir << std::endl;
    };

    /** @name Metadata */
    //@{
    /**
     * @brief Returns the identifying name of the VolumePkg.
     * @return Name of the VolumePkg
     */
    std::string getPkgName() const;

    /**
     * @brief Returns the VolumePkg version.
     *
     * Use in conjunction with volcart::VersionLibrary to verify the presence of
     * specific VolumePkg metadata keys.
     *
     * @return Version number of VolumePkg
     */
    int getVersion() const;  // Changed type from double to int

    /**
     * @brief Returns the boolean value of the VolumePkg read-only flag.
     *
     * When `true`, metadata values cannot be edited and slice data cannot be
     * added to the VolumePkg.
     */
    bool readOnly() const { return _readOnly; };

    /**
     * @brief Set/unset the VolumePkg read-only flag.
     * @param b Boolean representing new value of read-only flag
     */
    void readOnly(bool b) { _readOnly = b; };

    /**
     * @brief Sets the value of `key` in the VolumePkg metadata.
     *
     * These values are only stored in memory until saveMetadata() is called.
     * If VolumePkg is set to read-only, value is not set and function returns
     * `EXIT_SUCCESS`.
     *
     * @param key Metadata key identifier
     * @param value Value to be stored
     * @return `EXIT_SUCCESS` or `EXIT_FAILURE`
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

    /**
     * @brief Saves the metadata to the VolumePkg (.volpkg) file.
     */
    void saveMetadata();

    /**
     * @brief Saves the metadata to a user-specified location.
     * @param filePath Path to output file
     */
    void saveMetadata(const boost::filesystem::path& filePath);
    //@}

    /** @name Volume Data */
    //@{
    /**
     * @brief Returns the Volume object that stores slice data.
     * @return Reference to the volcart::Volume for this VolumePkg
     * @see common/types/Volume.h
     */
    const volcart::Volume& volume() const { return vol_; }

    /** @copydoc VolumePkg::volume() const */
    volcart::Volume& volume() { return vol_; }

    /**
     * @brief Returns the width of the slice images.
     *
     * This number is retrieved from the metadata and is not validated
     * against the slices stored in the .volpkg file.
     */
    int getSliceWidth() const;

    /**
     * @brief Returns the height of the slice images.
     *
     * This number is retrieved from the metadata and is not validated
     * against the slices stored in the .volpkg file.
     */
    int getSliceHeight() const;

    /**
     * @brief Returns the number of slice images.
     *
     * This number is retrieved from the metadata and is not validated
     * against the slices stored in the .volpkg file.
     */
    int getNumberOfSlices() const;

    /**
     * @brief Returns the size of voxels in microns (um).
     *
     * This is the "real-world" size of voxels. Only isometric voxels (voxels
     * with equal edge lengths) are supported.
     * @return Voxel size in microns (um)
     */
    double getVoxelSize() const;

    /**
     * @brief Returns the approx. thickness of a material layer in microns (um).
     *
     * This value is approximated by the user when the VolumePkg is created.
     * This is an intrinsic property of the scanned object and is therefore
     * indepedent of scan resolution. The material thickness in microns can be
     * used to estimate the material thickness in voxels for scans of any
     * resolution.
     *
     * \f[
        \frac{\mbox{Material Thickness }(um)}{\mbox{Voxel Size }(um)}
        = \mbox{Material Thickness }(voxels)
      \f]
     *
     * @return Layer thickness, measured in microns (um).
     */
    double getMaterialThickness() const;

    /**
     * @brief Sets the slice data for z-index `index` in the volume.
     *
     * Does nothing if VolumePkg read-only flag is set.
     * @warning This function will overwrite slice images stored in the .volpkg
     * file. Should only be used when constructing a new VolumePkg.
     * @param index Z-index of slice data
     * @param slice Image data
     * @return Boolean for write success/failure
     */
    bool setSliceData(size_t index, const cv::Mat& slice);
    //@}

    /** @name Segmentation Data */
    //@{
    /**
     * @brief Creates a new segmentation.
     *
     * Populates the .volpkg file with a new segmentation directory and adds the
     * ID to the internal list of segmentations.
     * @return Identifier name of the new segmentation
     */
    std::string newSegmentation();

    /**
     * @brief Returns the list of Segmentation IDs for the VolumePkg.
     *
     * IDs in this list can be passed to setActiveSegmentation() in order to
     * access data from a specific segmentation.
     * @return List of segmentation IDs
     */
    std::vector<std::string> getSegmentations() const;

    /**
     * @brief Sets the active segmentation.
     *
     * Data access functions like openCloud() and getMesh() return data from the
     * active segmentation. To get data from other segmentations, you must first
     * change the active segmentation using this function.
     *
     * @param id Segmentation ID of desired active segmentation
     */
    void setActiveSegmentation(const std::string& id);

    /**
     * @brief Returns the ID of the active segmentation.
     * @return Segmentation ID of active segmentation
     */
    std::string getActiveSegmentation();

    /**
     * @brief Returns the directory path for the active segmentation.
     *
     * This path can be absolute or relative.
     *
     * @return Directory path to the active segmentation
     */
    boost::filesystem::path getActiveSegPath();

    /**
     * @brief Returns the OrderedPointSet for the active segmentation.
     *
     * This returns a point cloud that represents segmented surface positions
     * within the Volume. An OrderedPointSet provides 2D access to these points.
     *
     * @return Segmented surface as an OrderedPointSet
     * @see common/types/OrderedPointSet.h
     * @see common/types/PointSet.h
     */
    volcart::OrderedPointSet<volcart::Point3d> openCloud() const;
    //@}

    /** @name Render Data */
    //@{
    /**
     * Gets the file path where the mesh for the active segmentation
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
    //@}

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

    /**
     * This is the segmentation that is currently being worked on
     */
    std::string activeSeg;

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
