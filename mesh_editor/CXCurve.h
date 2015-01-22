// CXCurve.h
// Chao Du 2014 Dec
#ifndef _CXCURVE_H_
#define _CXCURVE_H_

#include "mathUtils.h"

#include <vector>

namespace ChaoVis {

class CXCurve {

public:
    CXCurve( void );
	CXCurve( const CXCurve &nSrc );
    ~CXCurve( void );

	void SetSliceIndex( int nIndex ) { fSliceIndex = nIndex; }
	int GetSliceIndex( void ) { return fSliceIndex; }

	size_t GetPointsNum( void ) const { return fPoints.size(); }
	
	Vec2< float > GetPoint( int nIndex ) const { return fPoints[ nIndex ]; }
	void SetPoint( int nIndex, const Vec2< float > &nPt ) { fPoints[ nIndex ] = nPt; }

	int Get3DIndex( int nIndex ) const { return f3DPointsIndex[ nIndex ]; }
	void Set3DIndex( int nIndex, int n3DIndex ) { f3DPointsIndex[ nIndex ] = n3DIndex; }
	
	void InsertPoint( const Vec2< float > &nPt ) { fPoints.push_back( nPt ); }
	void Insert3DIndex( int n3DIndex ) { f3DPointsIndex.push_back( n3DIndex ); }

protected:

private:
	std::vector< Vec2< float > > fPoints; // actual 2D points
	std::vector< int > f3DPointsIndex; // mapping to 3D points index
	int fSliceIndex;

}; // class CXCurve

} // namespace ChaoVis

#endif // _CXCURVE_H_
