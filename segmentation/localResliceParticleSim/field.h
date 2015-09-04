#ifndef _DEMO_FIELD_
#define _DEMO_FIELD_

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>

#include "volumepkg.h"
#include "slice.h"

namespace DEMO {

class Field {
public:
    Field(VolumePkg *);

    uint16_t interpolate_at(cv::Vec3f);

    Slice reslice(cv::Vec3f, cv::Vec3f, cv::Vec3f, int32_t=64, int32_t=64);

    Slice resliceRadial(cv::Vec3f, cv::Vec3f, double, int32_t=64, int32_t=64);

private:
    VolumePkg* _volpkg;
    std::vector<cv::Mat> _field;
};

}

#endif
