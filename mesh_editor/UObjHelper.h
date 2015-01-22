// UObjHelper.h
// Chao Du 2014 Dec
#ifndef _UOBJHELPER_H_
#define _UOBJHELPER_H_

namespace ChaoVis {

bool ReadObjFile( const char *nFileName,
                  int *nNumVertex,
                  int *nNumFace,
                  float **nV,    /* v1x v1y v1z v2x v2y v2z ... */
                  int **nF,      /* f1a f1b f1c f2a f2b f2c ... */
                  float **nUV );

} // namespace ChaoVis

#endif // _UOBJHELPER_H_
