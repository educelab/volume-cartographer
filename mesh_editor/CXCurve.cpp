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
