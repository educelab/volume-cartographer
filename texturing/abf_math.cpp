//
// Created by Seth Parker on 6/10/16.
//

#include "abf_math.h"

// Returns the angle between ab and ac
double vec_angle( volcart::QuadPoint A, volcart::QuadPoint B, volcart::QuadPoint C ) {
  cv::Vec3d vec_1, vec_2;

  vec_1[0] = B[0] - A[0];
  vec_1[1] = B[1] - A[1];
  vec_1[2] = B[2] - A[2];

  vec_2[0] = C[0] - A[0];
  vec_2[1] = C[1] - A[1];
  vec_2[2] = C[2] - A[2];

  cv::norm(vec_1);
  cv::norm(vec_2);

  double dot = vec_1.dot(vec_2);

  if ( dot <= -1.0f )
    return M_PI;
  else if ( dot >= 1.0f )
    return 0.0f;
  else
    return acos(dot);

}