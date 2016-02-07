// Field object maintains a sliding window of loaded normal vectors in the
// volume
// Normals are read from disk when they're needed and can be deleted with
// clean()
// to keep the memory used to a minimum.

#ifndef _FIELD_
#define _FIELD_

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>

#include "volumepkg.h"

class Field
{
public:
    Field(VolumePkg& volpkg);
    ~Field();
    void clean();
    cv::Vec3f interpolate_at(const cv::Vec3f point) const;

private:
    VolumePkg& _volpkg;
    // This stores the vector field of surface normals.
    // Field coordinates are parallel to volume coordinates
    // field[z][x][y] == vol[x][y][z]
    cv::Vec3f*** _field;
    // Largest dimension of the CT slices
    int _blocksize;
    // Slices not in this set will be removed by clean()
    std::set<int32_t> _indexes_used_since_last_clean;
    void loadSlice(const int32_t index);  // To-Do: Is this name clear enough?
};

#endif
