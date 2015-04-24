#ifndef _REGION_
#define _REGION_

#include <list>
#include <queue>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>

#include "voxel.h"

extern Voxel**** volume;

typedef enum {
  CONNECTOR = 'KNCT',
  GRAV_ONLY = 'GRAV'
} regionMetric;

class Region {
  std::queue<Voxel*> live;
  std::list<Voxel*> dead;
public:
  Region(Voxel* v);
  void insert(Voxel* v);
  int growWith(regionMetric);
  void write();
};

#endif // _REGION_
