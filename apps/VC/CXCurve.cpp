// CXCurve.cpp
// Chao Du 2014 Dec
#include "CXCurve.h"

using namespace ChaoVis;

// Constructor
CXCurve::CXCurve( void )
{
}

// Copy constructor
CXCurve::CXCurve( const CXCurve &nSrc )
{
	fSliceIndex = nSrc.fSliceIndex;
	fPoints = nSrc.fPoints;
	f3DPointsIndex = nSrc.f3DPointsIndex;
}

// Desctructor
CXCurve::~CXCurve( void )
{
}

// Change point position, use different weighting function
void CXCurve::SetPointByDifference( int nIndex,
                                   const Vec2< float > &nDiff,
                                   double ( *ImpactFunc )( double, double, double ),
                                   int nImpactRange )
{
  for (int i = 0; i <= nImpactRange; ++i) {
    // The following should be simplified, but every time I've tried, it does weird things when dragging points. - SP, 2016
    if (nIndex - i >= 0) {
      SetPoint(nIndex - i, fPoints[nIndex - i] + nDiff * ImpactFunc(1.0, (double) i / nImpactRange, 1.0));
    }
    if (i == 0) {
      continue;
    }
    if (nIndex + i < fPoints.size()) {
      SetPoint(nIndex + i, fPoints[nIndex + i] + nDiff * ImpactFunc(1.0, (double) i / nImpactRange, 1.0));
    }
  }
}
