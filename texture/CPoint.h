// CPoint.h
// Chao Du Sept. 2014
#ifndef _CPOINT_H_
#define _CPOINT_H_

#include "CVec.h"

namespace ChaoVis {

template< size_t s, typename T >
class CPoint : public CVec< s, T > {

public:

private:

}; // class CPoint

typedef CPoint< 2, char   > CPoint2c;
typedef CPoint< 2, int    > CPoint2i;
typedef CPoint< 2, float  > CPoint2f;
typedef CPoint< 2, double > CPoint2d;

typedef CPoint< 3, char   > CPoint3c;
typedef CPoint< 3, int    > CPoint3i;
typedef CPoint< 3, float  > CPoint3f;
typedef CPoint< 3, double > CPoint3d;

} // namespace ChaoVis

#endif // _CPOINT_H_
