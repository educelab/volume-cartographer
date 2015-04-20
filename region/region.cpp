#include "region.h"

Region::Region(Voxel* v) {
  regionv.push_back(v);
  Vector pos = v->pos();
  int x = pos(0);
  int y = pos(1);
  int z = pos(2);
  volume[x][y][z] = NULL;
}

void Region::insert(Voxel* v) {
  regionv.push_back(v);
  Vector pos = v->pos();
  int x = pos(0);
  int y = pos(1);
  int z = pos(2);
  volume[x][y][z] = NULL;
}

int Region::grow() {
  int regsize = regionv.size();
  int counter = 0;
  for (int i = 0; i < regsize; ++i) {
    Vector pos = regionv[i]->pos();
    int x = pos(0);
    int y = pos(1);
    int z = pos(2);
    for (int loci = 0; loci < 27; ++loci) {
      int dx = (loci / 9) - 1;
      int dy = ((loci % 9) / 3) - 1;
      int dz = ((loci % 9) % 3) - 1;
      if (volume[x + dx][y + dy][z + dz] != NULL &&
          connector(*(regionv[i]), *(volume[x + dx][y + dy][z + dz]))) {
        // std::cout << "adding " << *(volume[x + dx][y + dy][z + dz]) << std::endl;
        regionv.push_back(volume[x + dx][y + dy][z + dz]);
        volume[x + dx][y + dy][z + dz] = NULL;
        counter++;
      }
    }
  }
  return counter;
}

void Region::write() {
  pcl::PointCloud<pcl::PointXYZRGBNormal> cloud;
  for (int i = 0; i < regionv.size(); ++i) {
    pcl::PointXYZRGBNormal point;
    Vector pos = regionv[i]->pos();
    point.x = pos(0);
    point.y = pos(1);
    point.z = pos(2);
    Vector norm = regionv[i]->norm();
    point.normal[0] = norm(0);
    point.normal[1] = norm(1);
    point.normal[2] = norm(2);
    point.rgb = regionv[i]->eig();
    cloud.push_back(point);
  }
  pcl::io::savePCDFileBinaryCompressed("output.pcd", cloud);
}
