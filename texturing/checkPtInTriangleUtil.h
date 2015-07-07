// checkPtInTriangleUtil.h
// Chao 2015 April
#ifndef _CHECKPTINTRIANGLEUTIL_H_
#define _CHECKPTINTRIANGLEUTIL_H_

namespace checkPtInTriangleUtil {


// Data structure
// REVISIT - NOTE - can also use other data type to represent 3D vectors, such as cv::Vec3d.
typedef struct Vec3d_tag {
    double data[ 3 ];
} Vec;

typedef Vec Point;


// Basic vector operations
// operator+
inline Vec operator+( const Vec &nV1,
                      const Vec &nV2 )
{
    Vec aVec;
    for ( int i = 0; i < 3; ++i ) {
        aVec.data[ i ] = nV1.data[ i ] + nV2.data[ i ];
    }
    return aVec;
}

// operator-
inline Vec operator-( const Vec &nV1,
                      const Vec &nV2 )
{
    Vec aVec;
    for ( int i = 0; i < 3; ++i ) {
        aVec.data[ i ] = nV1.data[ i ] - nV2.data[ i ];
    }
    return aVec;
}

// Dot product
inline double Dot( const Vec &nV1,
                   const Vec &nV2 )
{
    double aResult = 0;
    for ( int i = 0; i < 3; ++i ) {
        aResult += nV1.data[ i ] * nV2.data[ i ];
    }
    return aResult;
}

// Cross product
inline Vec Cross( const Vec &nV1,
                  const Vec &nV2 )
{
    Vec aResult;
    aResult.data[ 0 ] = nV1.data[ 1 ] * nV2.data[ 2 ] - nV1.data[ 2 ] * nV2.data[ 1 ];
    aResult.data[ 1 ] = nV1.data[ 2 ] * nV2.data[ 0 ] - nV1.data[ 0 ] * nV2.data[ 2 ];
    aResult.data[ 2 ] = nV1.data[ 0 ] * nV2.data[ 1 ] - nV1.data[ 1 ] * nV2.data[ 0 ];
    return aResult;
}


// Check if the point is within the angle (note it could also be in the reverse direction)
// A is the starting point, AB and AC are the edges
bool IsPtBetweenVecs( const Point &nPt,
                      const Point &nA,
                      const Point &nB,
                      const Point &nC )
{
    Vec aAB = nB - nA;
    Vec aAC = nC - nA;
    Vec aAP = nPt - nA;
    return( ( Cross( aAB, aAP ).data[ 2 ] ) * ( Cross( aAC, aAP ).data[ 2 ] ) < 0 );
}

// Check is the point is inside the triangle
bool IsPtInTriangle( const Point &nPt,
                     const Point &nA,
                     const Point &nB,
                     const Point &nC )
{
    return( IsPtBetweenVecs( nPt, nB, nA, nC ) && IsPtBetweenVecs( nPt, nA, nB, nC ) );
}

} // checkPtInTriangleUtil

#endif // _CHECKPTINTRIANGLEUTIL_H_
