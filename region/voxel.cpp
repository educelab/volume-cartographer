#include "voxel.h"

Voxel::Voxel(Vector pos, Vector n, float e, unsigned short c) {
  position = pos;
  normal = n;
  eigen = e;
  color = c;
}

void Voxel::project() {
  Vector G(1,0,0);
  gravity = G - (normal.dot(G)/normal.dot(normal)) * normal;
  slice_intersect = Vector(0, -normal(2), normal(1));
  normalize(gravity);
  normalize(slice_intersect);
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

float Voxel::packedColor() {
  uint8_t intensity = (uint8_t)(255 * (color / 65535.0));
  uint32_t rgb = ((uint32_t)intensity << 16) | ((uint32_t)intensity << 8) | (uint32_t)intensity;
  return *(float*)&rgb;
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
