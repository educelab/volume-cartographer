// UObjHelper.h
// Chao Du 2014 Dec
#ifndef _UOBJHELPER_H_
#define _UOBJHELPER_H_

#include <vector>
#include "mathUtils.h"
#include "CPoint.h"

namespace ChaoVis {

bool ReadObjFile( const char *nFileName,
                  int *nNumVertex,
                  int *nNumFace,
                  float **nV,    /* v1x v1y v1z v2x v2y v2z ... */
                  int **nF,      /* f1a f1b f1c f2a f2b f2c ... */
                  float **nUV,
                  std::vector< ChaoVis::Vec3< int > > &nFaces,
                  std::vector< ChaoVis::PointXYZRGBNormal > &nPoints,
                  std::vector< std::vector< int > > &nVDupList );

bool SaveObjFile( const char *nFileName,
                  int nNumVertex,
				  int nNumFace,
				  float *nV,
				  float *nF,
				  float *nUV );

} // namespace ChaoVis

#endif // _UOBJHELPER_H_
