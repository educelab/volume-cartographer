#include "voxel.h"

Voxel::Voxel(Vector pos, Vector n, float e) {
  position = pos;
  normal = n;
  eigen = e;
}

void Voxel::project() {
  Vector G(1,0,0);
  gravity = G - (normal.dot(G)/normal.dot(normal)) * normal;
  slice_intersect = Vector(0, -normal(2), normal(1));
  normalize(gravity);
  normalize(slice_intersect);
}

static void vround(Vector& v) {
  v = Vector(round(v(0)), round(v(1)), round(v(2)));
}

bool connector(Voxel x, Voxel y) {
  x.project();
  y.project();

  // look for a connection based on gravity
  Vector positive_gravity_to_x_ = y.position + y.gravity;
  Vector negative_gravity_to_x_ = y.position - y.gravity;
  vround(positive_gravity_to_x_);
  vround(negative_gravity_to_x_);
  Vector positive_gravity_to_y_ = x.position + x.gravity;
  Vector negative_gravity_to_y_ = x.position - x.gravity;
  vround(positive_gravity_to_y_);
  vround(negative_gravity_to_y_);
    
  // look for connection on slice intersection
  Vector positive_intersect_to_x_ = y.position + y.slice_intersect;
  Vector negative_intersect_to_x_ = y.position - y.slice_intersect;
  vround(positive_intersect_to_x_);
  vround(negative_intersect_to_x_);
  Vector positive_intersect_to_y_ = x.position + x.slice_intersect;
  Vector negative_intersect_to_y_ = x.position - x.slice_intersect;
  vround(positive_intersect_to_y_);
  vround(negative_intersect_to_y_);

  if ((positive_gravity_to_x_ == x.position || negative_gravity_to_x_ == x.position) ||
      (negative_gravity_to_y_ == y.position || negative_gravity_to_y_ == y.position)) {
    return true;
  }

  // if ((positive_intersect_to_x_ == x.position || negative_intersect_to_x_ == x.position) &&
  //     (positive_intersect_to_y_ == y.position || negative_intersect_to_x_ == y.position)) {
  //   return true;
  // }
  return false;
}
  
Vector Voxel::pos() {
  return position;
}

Vector Voxel::norm() {
  return normal;
}

float Voxel::eig() {
  return eigen;
}

bool operator<(const Voxel& x, const Voxel& y) {
  return x.eigen < y.eigen;
}

Vector offset(Voxel x, Voxel y) {
  return x.position - y.position;
}

std::ostream& operator<<(std::ostream& os, const Voxel& v) {
  os << "Voxel " << v.position << " " << v.eigen;
  return os;
}
