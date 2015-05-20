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

void Voxel::overwriteEigen(double color)  {
  unsigned char c = (unsigned char)((color / 65535) * 255);
  uint32_t pack = (uint32_t)c << 16 | (uint32_t)c << 8 | (uint32_t)c;
  eigen = *(float*)&pack;
}

Vector Voxel::pos() {
  return position;
}

Vector Voxel::norm() {
  return normal;
}

Vector Voxel::grav() {
  return gravity;
}

Vector Voxel::slice() {
  return  slice_intersect;
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
