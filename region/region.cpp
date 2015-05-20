#include "region.h"

static void vround(Vector& v) {
  v = Vector(round(v(0)), round(v(1)), round(v(2)));
}

static bool connector(Voxel x, Voxel y) {
  x.project();
  y.project();

  // look for a connection based on gravity
  Vector positive_gravity_to_x_ = y.pos() + y.grav();
  Vector negative_gravity_to_x_ = y.pos() - y.grav();
  vround(positive_gravity_to_x_);
  vround(negative_gravity_to_x_);
  Vector positive_gravity_to_y_ = x.pos() + x.grav();
  Vector negative_gravity_to_y_ = x.pos() - x.grav();
  vround(positive_gravity_to_y_);
  vround(negative_gravity_to_y_);

  // look for connection on slice intersection
  Vector positive_intersect_to_x_ = y.pos() + y.slice();
  Vector negative_intersect_to_x_ = y.pos() - y.slice();
  vround(positive_intersect_to_x_);
  vround(negative_intersect_to_x_);
  Vector positive_intersect_to_y_ = x.pos() + x.slice();
  Vector negative_intersect_to_y_ = x.pos() - x.slice();
  vround(positive_intersect_to_y_);
  vround(negative_intersect_to_y_);

  if ((positive_gravity_to_x_ == x.pos() || negative_gravity_to_x_ == x.pos()) ||
      (positive_gravity_to_y_ == y.pos() || negative_gravity_to_y_ == y.pos())) {
    return true;
  }

  if ((positive_intersect_to_x_ == x.pos() || negative_intersect_to_x_ == x.pos()) &&
      (positive_intersect_to_y_ == y.pos() || negative_intersect_to_y_ == y.pos())) {
    return true;
  }
  return false;
}

static bool grav_only(Voxel x, Voxel y) {
  Vector positive_gravity_to_x_ = y.pos() + y.grav();
  Vector negative_gravity_to_x_ = y.pos() - y.grav();
  vround(positive_gravity_to_x_);
  vround(negative_gravity_to_x_);
  Vector positive_gravity_to_y_ = x.pos() + x.grav();
  Vector negative_gravity_to_y_ = x.pos() - x.grav();
  vround(positive_gravity_to_y_);
  vround(negative_gravity_to_y_);
  if ((positive_gravity_to_x_ == x.pos() || negative_gravity_to_x_ == x.pos()) ||
      (positive_gravity_to_y_ == y.pos() || negative_gravity_to_y_ == y.pos())) {
    return true;
  }
  return false;
}

Region::Region(Voxel* v) {
  live.push(v);
  Vector pos = v->pos();
  int x = pos(0);
  int y = pos(1);
  int z = pos(2);
  volume[x][y][z] = NULL;
}

void Region::insert(Voxel* v) {
  live.push(v);
  Vector pos = v->pos();
  int x = pos(0);
  int y = pos(1);
  int z = pos(2);
  volume[x][y][z] = NULL;
}

int Region::growWith(regionMetric m) {
  bool (*metric)(Voxel,Voxel);

  switch(m) {
  case CONNECTOR:
    metric = &connector;
    break;
  case GRAV_ONLY:
    metric = &grav_only;
    break;
  default:
    std::cout << "ILLEGAL REGION METRIC" << std::endl;
    exit(EXIT_FAILURE);
  }

  int counter = 0;
  std::list<Voxel*> edge;
  while (!live.empty()) {
    Voxel* it = live.front();
    live.pop();
    dead.push_back(it);

    Vector pos = it->pos();
    int x = pos(0);
    int y = pos(1);
    int z = pos(2);

    for (int loci = 0; loci < 27; ++loci) {
      int dx = (loci / 9) - 1;
      int dy = ((loci % 9) / 3) - 1;
      int dz = ((loci % 9) % 3) - 1;

      if (volume[x + dx][y + dy][z + dz] != NULL &&
          metric(*it, *(volume[x + dx][y + dy][z + dz]))) {

        edge.push_back(volume[x + dx][y + dy][z + dz]);
        volume[x + dx][y + dy][z + dz] = NULL;

        counter++;
      }
    }
  }

  for (std::list<Voxel*>::iterator it = edge.begin(); it != edge.end(); ++it) {
    live.push(*it);
  }

  return counter;
}

void Region::texture(std::vector<cv::Mat>& m) {
  while (!live.empty()) {
    dead.push_back(live.front());
  }

  for (std::list<Voxel*>::iterator it = dead.begin(); it != dead.end(); ++it) {
    double color = textureWithMethod((*it)->pos(), (*it)->norm(), m, EFilterOption::FilterOptionIntersection);
    (*it)->overwriteEigen(color);
  }  
}

void Region::write() {
  pcl::PointCloud<pcl::PointXYZRGBNormal> cloud;
  while (!live.empty()) {
    pcl::PointXYZRGBNormal point;
    Voxel* it = live.front();
    live.pop();
    Vector pos = it->pos();
    point.x = pos(0);
    point.y = pos(1);
    point.z = pos(2);
    Vector norm = it->norm();
    point.normal[0] = norm(0);
    point.normal[1] = norm(1);
    point.normal[2] = norm(2);
    point.rgb = it->eig();
    cloud.push_back(point);
  }
  for (std::list<Voxel*>::iterator it = dead.begin(); it != dead.end(); ++it) {
    pcl::PointXYZRGBNormal point;
    Vector pos = (*it)->pos();
    point.x = pos(0);
    point.y = pos(1);
    point.z = pos(2);
    Vector norm = (*it)->norm();
    point.normal[0] = norm(0);
    point.normal[1] = norm(1);
    point.normal[2] = norm(2);
    point.rgb = (*it)->eig();
    cloud.push_back(point);
  }
  pcl::io::savePCDFileBinaryCompressed("output.pcd", cloud);
}
