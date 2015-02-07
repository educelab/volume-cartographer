// mathUtils.h
// Chao Du 2014 Nov
#ifndef _MATHUTILS_H_
#define _MATHUTILS_H_

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define TOO_SMALL (1e-6)

#ifndef M_PI
#define M_PI (3.1415926535897932384626)
#endif // M_PI

namespace ChaoVis {

template < typename T >
class Vec2 {
private:
	T val[ 2 ];
public:
	Vec2( void ) { val[ 0 ] = T( 0 ); val[ 1 ] = T( 0 ); }
	Vec2( const Vec2 &nCopy ) { val[ 0 ] = nCopy[ 0 ]; val[ 1 ] = nCopy[ 1 ]; }
	Vec2( T v1, T v2 ) { val[ 0 ] = v1; val[ 1 ] = v2; }

	const T operator[]( int i ) const { return val[ i ]; }
	T& operator[]( int i ) { return val[ i ]; }	// left value, http://stackoverflow.com/questions/6692982/how-to-assign-value-to-left-side-using-overload-operator
	void operator+=( const Vec2< T > &nRHS ) { val[ 0 ] += nRHS[ 0 ]; val[ 1 ] += nRHS[ 1 ]; }
    Vec2< T > operator+( const Vec2< T > &nRHS ) const { return Vec2< T >( val[ 0 ] + nRHS[ 0 ], val[ 1 ] + nRHS[ 1 ] ); }
    Vec2< T > operator-( const Vec2< T > &nRHS ) const { return Vec2< T >( val[ 0 ] - nRHS[ 0 ], val[ 1 ] - nRHS[ 1 ] ); }
    Vec2< T > operator*( T nRHS ) const { return Vec2< T >( val[ 0 ] * nRHS, val[ 1 ] * nRHS ); }
};

template < typename T >
class Vec3 {
private:
	T val[ 3 ];
public:
	Vec3( void ) { val[ 0 ] = T( 0 ); val[ 1 ] = T( 0 ); val[ 2 ] = T( 0 ); }
	Vec3( const Vec3 &nCopy ) { val[ 0 ] = nCopy[ 0 ]; val[ 1 ] = nCopy[ 1 ]; val[ 2 ] = nCopy[ 2 ]; }
	Vec3( T val0, T val1, T val2 ) { val[ 0 ] = val0; val[ 1 ] = val1; val[ 2 ] = val2; }

	const T operator[]( int i ) const { return val[ i ]; }
	T& operator[]( int i ) { return val[ i ]; }
	void operator+=( const Vec3< T > &nRHS ) { val[ 0 ] += nRHS[ 0 ]; val[ 1 ] += nRHS[ 1 ]; val[ 2 ] += nRHS[ 2 ]; }
	void operator-=( const Vec3< T > &nRHS ) { val[ 0 ] -= nRHS[ 0 ]; val[ 1 ] -= nRHS[ 1 ]; val[ 2 ] -= nRHS[ 2 ]; }
	Vec3< T >& operator*=( T nVal ) { val[ 0 ] *= nVal; val[ 1 ] *= nVal; val[ 2 ] *= nVal; return *this; }
    Vec3< T > operator*( T nVal ) { return Vec3< T >( val[ 0 ] * nVal, val[ 1 ] * nVal, val[ 2 ] * nVal ); }
    Vec3< T > operator+( const Vec3< T > &nRHS ) { return Vec3< T >( val[ 0 ] + nRHS[ 0 ], val[ 1 ] + nRHS[ 1 ], val[ 2 ] + nRHS[ 2 ] ); }
    Vec3< T > operator-( const Vec3< T > &nRHS ) { return Vec3< T >( val[ 0 ] - nRHS[ 0 ], val[ 1 ] - nRHS[ 1 ], val[ 2 ] - nRHS[ 2 ] ); }
};

template < typename T >
class Vec4 {
private:
	T val[ 4 ];
public:
	Vec4( void ) { val[ 0 ] = T( 0 ); val[ 1 ] = T( 0 ); val[ 2 ] = T( 0 ); val[ 3 ] = T( 0 ); }
	Vec4( const Vec4 &nCopy ) { val[ 0 ] = nCopy[ 0 ]; val[ 1 ] = nCopy[ 1 ]; val[ 2 ] = nCopy[ 2 ]; val[ 3 ] = nCopy[ 3 ]; }

	const T operator[]( int i ) const { return val[ i ]; }
	T& operator[]( int i ) { return val[ i ]; }
	void operator+=( const Vec4< T > &nRHS ) { val[ 0 ] += nRHS[ 0 ]; val[ 1 ] += nRHS[ 1 ]; val[ 2 ] += nRHS[ 2 ]; val[ 3 ] += nRHS[ 3 ]; }
};


// dot product
// REVISIT - make this a member function
template < typename T >
inline T Dot( const Vec2< T > &v1, const Vec2< T > &v2 )
{
    return( v1[ 0 ] * v2[ 0 ] + v1[ 1 ] * v2[ 1 ] );
}
template < typename T >
T Dot( const Vec3< T > &v1, const Vec3< T > &v2 )
{
    return( v1[ 0 ] * v2[ 0 ] + v1[ 1 ] * v2[ 1 ] + v1[ 2 ] * v2[ 2 ] );
}
template < typename T >
T Dot( const Vec4< T > &v1, const Vec4< T > &v2 )
{
    return( v1[ 0 ] * v2[ 0 ] + v1[ 1 ] * v2[ 1 ] + v1[ 2 ] * v2[ 2 ] + v1[ 3 ] * v2[ 3 ] );
}

template < typename T >
class Mat22 {
private:
	Vec2< T > val[ 2 ]; // row
public:
	Mat22< T >( void ) {}
	Mat22< T >( const Mat22< T > &nCopy ) { for ( int i = 0; i < 2; ++i ) { val[ i ] = nCopy[ i ]; } }
	Mat22< T >( T a11, T a12, T a21, T a22 ) { val[ 0 ][ 0 ] = a11; val[ 0 ][ 1 ] = a12; val[ 1 ][ 0 ] = a21; val[ 1 ][ 1 ] = a22; }

	const Vec2< T > operator[]( int i ) const { return val[ i ]; }
	Vec2< T >& operator[]( int i ) { return val[ i ]; }
	Vec2< T > operator*( const Vec2< T > &nRHS )
	{
		Vec2< T > aResult;
		aResult[ 0 ] = Dot< T >( val[ 0 ], nRHS );
		aResult[ 1 ] = Dot< T >( val[ 1 ], nRHS );
		return aResult;
	}

	Mat22< T > operator*( const Mat22< T > &nRHS )
	{
		Mat22< T > aResult;
		for ( int i = 0; i < 2; ++i ) {
			for ( int j = 0; j < 2; ++j ) {
				aResult[ i ][ j ] = 0.0;
				for ( int k = 0; k < 2; ++k ) {
					aResult[ i ][ j ] += val[ i ][ k ] * nRHS[ k ][ j ];
				}
			}
		}
		return aResult;
	}

    // REVISIT - ERROR - C2102: '&' requires l-value
//    const T* constData( void ) const { return &( val[ 0 ] ); }
};

template < typename T >
class Mat33 {
private:
	Vec3< T > val[ 3 ]; // row
public:
	Mat33< T >( void ) {}
	Mat33< T >( const Mat33< T > &nCopy ) { for ( int i = 0; i < 3; ++i ) { val[ i ] = nCopy[ i ]; } }

	const Vec3< T > operator[]( int i ) const { return val[ i ]; }
	Vec3< T >& operator[]( int i ) { return val[ i ]; }
	Vec3< T > operator*( const Vec3< T > &nRHS )
	{
		Vec3< T > aResult;
		aResult[ 0 ] = Dot< T >( val[ 0 ], nRHS );
		aResult[ 1 ] = Dot< T >( val[ 1 ], nRHS );
		aResult[ 2 ] = Dot< T >( val[ 2 ], nRHS );
		return aResult;
	}

	Mat33< T > operator*( const Mat33< T > &nRHS )
	{
		Mat33< T > aResult;
		for ( int i = 0; i < 3; ++i ) {
			for ( int j = 0; j < 3; ++j ) {
				aResult[ i ][ j ] = 0.0;
				for ( int k = 0; k < 3; ++k ) {
					aResult[ i ][ j ] += val[ i ][ k ] * nRHS[ k ][ j ];
				}
			}
		}
		return aResult;
	}

    // REVISIT - ERROR - C2102: '&' requires l-value
//    const T* constData( void ) const { return &val[ 0 ][ 0 ]; }
};

template < typename T >
class Mat44 {
private:
	Vec4< T > val[ 4 ]; // row
public:
	Mat44< T >( void ) {}
	Mat44< T >( const Mat44< T > &nCopy ) { for ( int i = 0; i < 4; ++i ) { val[ i ] = nCopy[ i ]; } }

	const Vec4< T > operator[]( int i ) const { return val[ i ]; }
	Vec4< T >& operator[]( int i ) { return val[ i ]; }

    // REVISIT - ERROR - C2102: '&' requires l-value
//    const T* constData( void ) const { return &( val[ 0 ] ); }
};

// cross product, only for 3-vector
template < typename T >
Vec3< T > Cross( const Vec3< T > &u, const Vec3< T > &v )
{
	Vec3< T > aResult;
	aResult[ 0 ] = T( u[ 1 ] * v[ 2 ] - u[ 2 ] * v[ 1 ] );
	aResult[ 1 ] = T( u[ 2 ] * v[ 0 ] - u[ 0 ] * v[ 2 ] );
	aResult[ 2 ] = T( u[ 0 ] * v[ 1 ] - u[ 1 ] * v[ 0 ] );
	return aResult;
}

// norm
// REVISIT - make this a member function
template < typename T >
T Norm( const Vec2< T > &v )
{
	return( sqrt( v[ 0 ] * v[ 0 ] + v[ 1 ] * v[ 1 ] ) );
}
template < typename T >
T Norm( const Vec3< T > &v )
{
	return( sqrt( v[ 0 ] * v[ 0 ] + v[ 1 ] * v[ 1 ] + v[ 2 ] * v[ 2 ] ) );
}
template < typename T >
T Norm( const Vec4< T > &v )
{
	return( sqrt( v[ 0 ] * v[ 0 ] + v[ 1 ] * v[ 1 ] + v[ 2 ] * v[ 2 ] + v[ 3 ] * v[ 3 ] ) );
}

// normalize
// REVISIT - check division
template < typename T >
Vec2< T > Normalize( const Vec2< T > &v )
{
	Vec2< T > aResult;
	T aFactor = Norm< T >( v );
	if ( fabs( aFactor ) < TOO_SMALL ) {
		aResult[ 0 ] = aResult[ 1 ] = T( 0.0 );
		return aResult;
	}
	aResult[ 0 ] = T( v[ 0 ] / aFactor );
	aResult[ 1 ] = T( v[ 1 ] / aFactor );
	return aResult;
}
template < typename T >
Vec3< T > Normalize( const Vec3< T > &v )
{
	Vec3< T > aResult;
	T aFactor = Norm< T >( v );
	if ( fabs( aFactor ) < TOO_SMALL ) {
		aResult[ 0 ] = aResult[ 1 ] = aResult[ 2 ] = T( 0.0 );
		return aResult;
	}
	aResult[ 0 ] = T( v[ 0 ] / aFactor );
	aResult[ 1 ] = T( v[ 1 ] / aFactor );
	aResult[ 2 ] = T( v[ 2 ] / aFactor );
	return aResult;
}
template < typename T >
Vec4< T > Normalize( const Vec4< T > &v )
{
	Vec4< T > aResult;
	T aFactor = Norm< T >( v );
	if ( fabs( aFactor ) < TOO_SMALL ) {
		aResult[ 0 ] = aResult[ 1 ] = aResult[ 2 ] = aResult[ 3 ] = T( 0.0 );
		return aResult;
	}
	aResult[ 0 ] = T( v[ 0 ] / aFactor );
	aResult[ 1 ] = T( v[ 1 ] / aFactor );
	aResult[ 2 ] = T( v[ 2 ] / aFactor );
	aResult[ 3 ] = T( v[ 3 ] / aFactor );
	return aResult;
}

template < typename T >
Mat33< T > RFromLookatAndUp( const Vec3< T > &nLookat,
							 const Vec3< T > &nUp )
{
	// REVISIT - this is weird! when we set up the view frustum (in view-frustum-rotation.v.glsl), we assume the camera looks
	//           at the -z direction, the up is +y, so the camera coordinates (regarding to rotation) is already NOT RIGHT-HAND
	//           coordinates!!! so we can not align it with the normal x y z axis to get the R, instead, we have to stick with
	//           its initial rotation (LEFT-HAND coordinates).
	Mat33< T > aR;
	Vec3< T > aRight = Cross< T >( nUp, nLookat );
	aR[ 0 ] = Normalize< T >( aRight );//Normalize< T >( /*aRight*/Vec3< T >( -aRight[ 0 ], -aRight[ 1 ], -aRight[ 2 ] ) );// Normalize< T >( Cross< T >( nUp, nLookat ) );
	aR[ 1 ] = Normalize< T >( nUp );//Normalize< T >( Vec3< T >( -nUp[ 0 ], -nUp[ 1 ], -nUp[ 2 ] ) );/* Normalize< T >( nUp );*/
	aR[ 2 ] = Normalize< T >( nLookat ); //Normalize< T >( nUp );/*Normalize< T >( Vec3< T >( -nLookat[ 0 ], -nLookat[ 1 ], -nLookat[ 2 ] ) );*/
	return aR;
}

template < typename T >
Mat33< T > RFromAngle( T alpha,		// angle about x
					   T beta,		// angle about y
					   T theta )	// angle about z
{
	Mat33< T > aR;
	Mat33< T > aX;
	aX[ 0 ][ 0 ] = 1.0; aX[ 0 ][ 1 ] = 0.0;          aX[ 0 ][ 2 ] = 0.0;
	aX[ 1 ][ 0 ] = 0.0; aX[ 1 ][ 1 ] = cos( alpha ); aX[ 1 ][ 2 ] = -sin( alpha );
	aX[ 2 ][ 0 ] = 0.0; aX[ 2 ][ 1 ] = sin( alpha ); aX[ 2 ][ 2 ] = cos( alpha );
	Mat33< T > aY;
	aY[ 0 ][ 0 ] = cos( beta ); aY[ 0 ][ 1 ] = 0.0; aY[ 0 ][ 2 ] = -sin( beta );
	aY[ 1 ][ 0 ] = 0.0;         aY[ 1 ][ 1 ] = 1.0; aY[ 1 ][ 2 ] = 0.0;
	aY[ 2 ][ 0 ] = sin( beta ); aY[ 2 ][ 1 ] = 0.0; aY[ 2 ][ 2 ] = cos( beta );
	Mat33< T > aZ;
	aZ[ 0 ][ 0 ] = cos( theta ); aZ[ 0 ][ 1 ] = -sin( theta ); aZ[ 0 ][ 2 ] = 0.0;
	aZ[ 1 ][ 0 ] = sin( theta ); aZ[ 1 ][ 1 ] = cos( theta );  aZ[ 1 ][ 2 ] = 0.0;
	aZ[ 2 ][ 0 ] = 0.0;          aZ[ 2 ][ 1 ] = 0.0;           aZ[ 2 ][ 2 ] = 1.0;

	aR = aX * aY * aZ;
	return aR;
}

// determinant 2x2
template < typename T >
T Det( const Mat22< T > &nSrc )
{
	return( nSrc[ 0 ][ 0 ] * nSrc[ 1 ][ 1 ] - nSrc[ 0 ][ 1 ] * nSrc[ 1 ][ 0 ] );
}

// determinant 3x3
template < typename T >
T Det( const Mat33< T > &nSrc )
{
	return( nSrc[ 0 ][ 0 ] * nSrc[ 1 ][ 1 ] * nSrc[ 2 ][ 2 ] - nSrc[ 0 ][ 0 ] * nSrc[ 2 ][ 1 ] * nSrc[ 1 ][ 2 ] +
			nSrc[ 0 ][ 1 ] * nSrc[ 2 ][ 0 ] * nSrc[ 1 ][ 2 ] - nSrc[ 0 ][ 1 ] * nSrc[ 1 ][ 0 ] * nSrc[ 2 ][ 2 ] + 
			nSrc[ 0 ][ 2 ] * nSrc[ 1 ][ 0 ] * nSrc[ 2 ][ 1 ] - nSrc[ 0 ][ 2 ] * nSrc[ 1 ][ 1 ] * nSrc[ 2 ][ 0 ] );
}

// determinant 4x4
template < typename T >
T Det( const Mat44< T > &nSrc )
{
	Mat33< T > A11, A12, A13, A14;
	A11[ 0 ][ 0 ] = nSrc[ 1 ][ 1 ]; A11[ 0 ][ 1 ] = nSrc[ 1 ][ 2 ]; A11[ 0 ][ 2 ] = nSrc[ 1 ][ 3 ];
	A11[ 1 ][ 0 ] = nSrc[ 2 ][ 1 ]; A11[ 1 ][ 1 ] = nSrc[ 2 ][ 2 ]; A11[ 1 ][ 2 ] = nSrc[ 2 ][ 3 ];
	A11[ 2 ][ 0 ] = nSrc[ 3 ][ 1 ]; A11[ 2 ][ 1 ] = nSrc[ 3 ][ 2 ]; A11[ 2 ][ 2 ] = nSrc[ 3 ][ 3 ];

	A12[ 0 ][ 0 ] = nSrc[ 1 ][ 0 ]; A12[ 0 ][ 1 ] = nSrc[ 1 ][ 2 ]; A12[ 0 ][ 2 ] = nSrc[ 1 ][ 3 ];
	A12[ 1 ][ 0 ] = nSrc[ 2 ][ 0 ]; A12[ 1 ][ 1 ] = nSrc[ 2 ][ 2 ]; A12[ 1 ][ 2 ] = nSrc[ 2 ][ 3 ];
	A12[ 2 ][ 0 ] = nSrc[ 3 ][ 0 ]; A12[ 2 ][ 1 ] = nSrc[ 3 ][ 2 ]; A12[ 2 ][ 2 ] = nSrc[ 3 ][ 3 ];

	A13[ 0 ][ 0 ] = nSrc[ 1 ][ 0 ]; A13[ 0 ][ 1 ] = nSrc[ 1 ][ 1 ]; A13[ 0 ][ 2 ] = nSrc[ 1 ][ 3 ];
	A13[ 1 ][ 0 ] = nSrc[ 2 ][ 0 ]; A13[ 1 ][ 1 ] = nSrc[ 2 ][ 1 ]; A13[ 1 ][ 2 ] = nSrc[ 2 ][ 3 ];
	A13[ 2 ][ 0 ] = nSrc[ 3 ][ 0 ]; A13[ 2 ][ 1 ] = nSrc[ 3 ][ 1 ]; A13[ 2 ][ 2 ] = nSrc[ 3 ][ 3 ];

	A14[ 0 ][ 0 ] = nSrc[ 1 ][ 0 ]; A14[ 0 ][ 1 ] = nSrc[ 1 ][ 1 ]; A14[ 0 ][ 2 ] = nSrc[ 1 ][ 2 ];
	A14[ 1 ][ 0 ] = nSrc[ 2 ][ 0 ]; A14[ 1 ][ 1 ] = nSrc[ 2 ][ 1 ]; A14[ 1 ][ 2 ] = nSrc[ 2 ][ 2 ];
	A14[ 2 ][ 0 ] = nSrc[ 3 ][ 0 ]; A14[ 2 ][ 1 ] = nSrc[ 3 ][ 1 ]; A14[ 2 ][ 2 ] = nSrc[ 3 ][ 2 ];

	return( nSrc[ 0 ][ 0 ] * Det( A11 ) - 
			nSrc[ 0 ][ 1 ] * Det( A12 ) +
			nSrc[ 0 ][ 2 ] * Det( A13 ) -
			nSrc[ 0 ][ 3 ] * Det( A14 ) );
}

// inverse 2x2
template < typename T >
Mat22< T > Inverse( const Mat22< T > &nSrc )
{
	Mat22< T > aResult;
	T aDeterminant = Det( nSrc );

	if ( fabs( aDeterminant ) < TOO_SMALL ) {
		printf( "ERROR: division by zero when invert a matrix\n" );
		exit( -1 );
	}

	aResult[ 0 ][ 0 ] = nSrc[ 0 ][ 0 ] / aDeterminant; aResult[ 0 ][ 1 ] = -nSrc[ 0 ][ 1 ] / aDeterminant;
	aResult[ 1 ][ 0 ] = -nSrc[ 1 ][ 0 ] / aDeterminant; aResult[ 1 ][ 1 ] = nSrc[ 0 ][ 0 ] / aDeterminant;

	return aResult;
}

// inverse 3x3
template < typename T >
Mat33< T > Inverse( const Mat33< T > &nSrc )
{
	Mat33< T > aResult;
	T aDeterminant = Det( nSrc );

	if ( fabs( aDeterminant ) < TOO_SMALL ) {
		printf( "ERROR: division by zero when invert a matrix\n" );
		exit( -1 );
	}

	aResult[ 0 ][ 0 ] =  Det( Mat22< T >( nSrc[ 1 ][ 1 ], nSrc[ 1 ][ 2 ], 
									 nSrc[ 2 ][ 1 ], nSrc[ 2 ][ 2 ] ) ) / aDeterminant;
	aResult[ 0 ][ 1 ] = -Det( Mat22< T >( nSrc[ 0 ][ 1 ], nSrc[ 0 ][ 2 ], 
									 nSrc[ 2 ][ 1 ], nSrc[ 2 ][ 2 ] ) ) / aDeterminant;
	aResult[ 0 ][ 2 ] =  Det( Mat22< T >( nSrc[ 0 ][ 1 ], nSrc[ 0 ][ 2 ], 
									 nSrc[ 1 ][ 1 ], nSrc[ 1 ][ 2 ] ) ) / aDeterminant;

	aResult[ 1 ][ 0 ] = -Det( Mat22< T >( nSrc[ 1 ][ 0 ], nSrc[ 1 ][ 2 ], 
				             				 nSrc[ 2 ][ 0 ], nSrc[ 2 ][ 2 ] ) ) / aDeterminant;
	aResult[ 1 ][ 1 ] =  Det( Mat22< T >( nSrc[ 0 ][ 0 ], nSrc[ 0 ][ 2 ], 
				             				 nSrc[ 2 ][ 0 ], nSrc[ 2 ][ 2 ] ) ) / aDeterminant;
	aResult[ 1 ][ 2 ] = -Det( Mat22< T >( nSrc[ 0 ][ 0 ], nSrc[ 0 ][ 2 ], 
									 nSrc[ 1 ][ 0 ], nSrc[ 1 ][ 2 ] ) ) / aDeterminant;

	aResult[ 2 ][ 0 ] =  Det( Mat22< T >( nSrc[ 1 ][ 0 ], nSrc[ 1 ][ 1 ], 
				             				 nSrc[ 2 ][ 0 ], nSrc[ 2 ][ 1 ] ) ) / aDeterminant;
	aResult[ 2 ][ 1 ] = -Det( Mat22< T >( nSrc[ 0 ][ 0 ], nSrc[ 0 ][ 1 ], 
				             				 nSrc[ 2 ][ 0 ], nSrc[ 2 ][ 1 ] ) ) / aDeterminant;
	aResult[ 2 ][ 2 ] =  Det( Mat22< T >( nSrc[ 0 ][ 0 ], nSrc[ 0 ][ 1 ], 
									 nSrc[ 1 ][ 0 ], nSrc[ 1 ][ 1 ] ) ) / aDeterminant;

	return aResult;
}

// inverse 4x4
template < typename T >
Mat44< T > Inverse( const Mat44< T > &nSrc )
{
	Mat44< T > aResult;
	//T aDeterminant = nSrc[ 0 ][ 0 ] * nSrc[ 1 ][ 1 ] - nSrc[ 0 ][ 1 ] * nSrc[ 1 ][ 0 ];

	//if ( fabs( aDeterminant ) < TOO_SMALL ) {
	//	printf( "ERROR: division by zero when invert a matrix\n" );
	//	exit( -1 );
	//}

	//aResult[ 0 ][ 0 ] = nSrc[ 0 ][ 0 ] / aDeterminant; aResult[ 0 ][ 1 ] = -nSrc[ 0 ][ 1 ] / aDeterminant;
	//aResult[ 1 ][ 0 ] = -nSrc[ 1 ][ 0 ] / aDeterminant; aResult[ 1 ][ 1 ] = nSrc[ 0 ][ 0 ] / aDeterminant;

	// REVISIT - Chao 20141204 - FILL ME HERE, not implemented
	// for A = [ R, t; 0, 1 ], A^-1 = [ R^-1, -R^-1 * t; 0, 1 ]
	printf( "ERROR: matrix 4x4 inversion not implemented\n" );

	return aResult;
}

// compose R (3x3) and t (3x1) into one matrix (4x4)
template <typename T >
Mat44< T > ComposeRT( const Mat33< T > &nR,
                      const Vec3< T > &nT )
{
    Mat44< T > aResult;

    for ( int i = 0; i < 3; ++i ) {
        for ( int j = 0; j < 3; ++j ) {
            aResult[ i ][ j ] = nR[ i ][ j ];
        }
        aResult[ i ][ 3 ] = nT[ i ];
    }
    aResult[ 3 ][ 0 ] = aResult[ 3 ][ 1 ] = aResult[ 3 ][ 2 ] = ( T )( 0.0 );
    aResult[ 3 ][ 0 ] = ( T )( 1.0 );

    return aResult;
}

} // namespace ChaoVis

#endif // _MATHUTILS_H_
