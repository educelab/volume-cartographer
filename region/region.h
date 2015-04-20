#ifndef _REGION_
#define _REGION_

#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>

#include "voxel.h"

extern Voxel**** volume;

class Region {
  std::vector<Voxel*> regionv;
public:
  Region(Voxel* v);
  void insert(Voxel* v);
  int grow();
  void write();
};

#endif // _REGION_
