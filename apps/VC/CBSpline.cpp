// CBSpline.cpp
// Chao Du 2015 April
#include "CBSpline.h"

#include "CMatrixMN.h"
#include "UVecMatOperations.h"

//#define _DEBUG

using namespace ChaoVis;

// Constructor
CBSpline::CBSpline( void )
{
}

// Destructor
CBSpline::~CBSpline( void )
{
}

// Set control points
void CBSpline::SetControlPoints( const std::vector< Vec2< double > > &nControlPoints )
{
    // REVISIT - this is a little bit waste of resources, can we incrementally add new control points?
    //           intuitively, we need to optimize the curve each time we change the control points, but is it
    //           possible that the new Bezier segment can be determined only by the newly added control point,
    //           say, maybe by the characteristic of Bezier curve?
    if ( fControlPoints.size() > 0 ) {
        fControlPoints.clear();
        fCurveSegments.clear();
    }

    for ( size_t i = 0; i < nControlPoints.size(); ++i ) {
        fControlPoints.push_back( nControlPoints[ i ] );
    }

    UpdateCurve();
}

// Set control points
void CBSpline::SetControlPoints( const std::vector< cv::Vec2f > &nControlPoints )
{
    // REVISIT - this is a little bit waste of resources, can we incrementally add new control points?
    //           intuitively, we need to optimize the curve each time we change the control points, but is it
    //           possible that the new Bezier segment can be determined only by the newly added control point,
    //           say, maybe by the characteristic of Bezier curve?
    if ( fControlPoints.size() > 0 ) {
        fControlPoints.clear();
        fCurveSegments.clear();
    }

    for ( size_t i = 0; i < nControlPoints.size(); ++i ) {
        fControlPoints.push_back( Vec2< double >( nControlPoints[ i ][ 0 ], nControlPoints[ i ][ 1 ] ) );
    }

    UpdateCurve();
}

// Get sample points
void CBSpline::GetSamplePoints( std::vector< Vec2< double > > &nSamplePoints )
{
    for ( size_t i = 0; i < fCurveSegments.size(); ++i ) {
        fCurveSegments[ i ].GetSamplePoints( nSamplePoints );
    }
}

// Get sample points
void CBSpline::GetSamplePoints( std::vector< cv::Vec2f > &nSamplePoints )
{
    for ( size_t i = 0; i < fCurveSegments.size(); ++i ) {
        fCurveSegments[ i ].GetSamplePoints( nSamplePoints );
    }
}

// Update curve
void CBSpline::UpdateCurve( void )
{
    // solve for control points for each Bezier curve segment
    // # of unknowns = # of curve segments = # of control points - 1
    int aNumUnknowns = ( int )fControlPoints.size() - 1;
    CVectorN< double > aX( aNumUnknowns ), aY( aNumUnknowns ), aB( aNumUnknowns );
    CMatrixMN< double > aA = Zero< double >( aNumUnknowns, aNumUnknowns );

    // set up matrix
    aA( 0, 0 ) = 2; aA( 0, 1 ) = 1;
    aA( aNumUnknowns - 1, aNumUnknowns - 2 ) = 2; aA( aNumUnknowns - 1, aNumUnknowns - 1 ) = 7;
    for ( int i = 1; i < aNumUnknowns - 1; ++i ) {
        aA( i, i - 1 ) = 1; aA( i, i ) = 4; aA( i, i + 1 ) = 1;
    }

#ifdef _DEBUG
    //aA.Dump();
#endif // _DEBUG

    // solve tri-diagonal matrix
    // REVISIT - note that we need to use the same left hand side but different right hand side,
    //           which is a practical problem, so MKL actually support such kind of requirement
    // for x component
    aB[ 0 ] = 2 * fControlPoints[ 1 ][ 0 ] + fControlPoints[ 0 ][ 0 ];
    aB[ aNumUnknowns - 1 ] = 8 * fControlPoints[ aNumUnknowns - 1 ][ 0 ] + fControlPoints[ aNumUnknowns ][ 0 ];
    for ( int i  = 1; i < aNumUnknowns - 1; ++i ) {
        aB[ i ] = 4 * fControlPoints[ i ][ 0 ] + 2 * fControlPoints[ i + 1 ][ 0 ];
    }
#ifdef _DEBUG
    //aB.Dump();
#endif // _DEBUG
    ThomasTriDiagonal< double >( aA, aB, aX );
#ifdef _DEBUG
    aX.Dump();
#endif // _DEBUG

    // for y component
    aB[ 0 ] = 2 * fControlPoints[ 1 ][ 1 ] + fControlPoints[ 0 ][ 1 ];
    aB[ aNumUnknowns - 1 ] = 8 * fControlPoints[ aNumUnknowns - 1 ][ 1 ] + fControlPoints[ aNumUnknowns ][ 1 ];
    for ( int i  = 1; i < aNumUnknowns - 1; ++i ) {
        aB[ i ] = 4 * fControlPoints[ i ][ 1 ] + 2 * fControlPoints[ i + 1 ][ 1 ];
    }
    ThomasTriDiagonal< double >( aA, aB, aY );
#ifdef _DEBUG
    aY.Dump();
#endif // _DEBUG

    // calculate P2,i
    CVectorN< double > aX2( aNumUnknowns ), aY2( aNumUnknowns );
    aX2[ aNumUnknowns - 1 ] = ( fControlPoints[ aNumUnknowns ][ 0 ] + aX[ aNumUnknowns - 1 ] ) / 2.0;
    aY2[ aNumUnknowns - 1 ] = ( fControlPoints[ aNumUnknowns ][ 1 ] + aY[ aNumUnknowns - 1 ] ) / 2.0;
    for ( int i = 0; i < aNumUnknowns - 1; ++i ) {
        aX2[ i ] = 2 * fControlPoints[ i + 1 ][ 0 ] - aX[ i + 1 ];
        aY2[ i ] = 2 * fControlPoints[ i + 1 ][ 1 ] - aY[ i + 1 ];
    }
#ifdef _DEBUG
    aX2.Dump();
    aY2.Dump();
#endif // _DEBUG

    std::vector< Vec2< double > > aControlPointsForSeg;
    aControlPointsForSeg.resize( 4 );
    for ( int i = 0; i < aNumUnknowns; ++i ) {
        CBezierCurve aCurveSegment;
        aControlPointsForSeg[ 0 ] = fControlPoints[ i ]; // P0,i = Ki
        aControlPointsForSeg[ 1 ] = Vec2< double >( aX[ i ], aY[ i ] ); // P1,i
        aControlPointsForSeg[ 2 ] = Vec2< double >( aX2[ i ], aY2[ i ] ); // P2,i
        aControlPointsForSeg[ 3 ] = fControlPoints[ i + 1 ]; // P3,i = P0,i-1 = Ki+1
        aCurveSegment.SetControlPoints( aControlPointsForSeg );
        fCurveSegments.push_back( aCurveSegment );
    }
}

// Draw the curve on image
void CBSpline::DrawOnImage( cv::Mat &nImg,
                            const cv::Scalar &nColor )
{
    for ( size_t i = 0; i < fCurveSegments.size(); ++i ) {
        fCurveSegments[ i ].DrawOnImage( nImg, nColor );
    }
}
