// C3DObj.h
// Chao Du 2014 Dec
#ifndef _C3DOBJ_H_
#define _C3DOBJ_H_

#include "mathUtils.h"

namespace ChaoVis {

template < typename T >
class C3DObj {

public:
    C3DObj( void );
	virtual ~C3DObj( void ) {}

    const Mat33< T >& GetRotation( void ) const { return fR; }
    void SetRotation( const Mat33< T > &nR ) { fR = nR; }
    const Vec3< T >& GetTranslation( void ) const { return fT; }
    void SetTranslation( const Vec3< T > &nT ) { fT = nT; }

	void ChangeTranslationByDifference( const Vec3< T > &nTD ) { fT += nTD; }

    virtual void Draw( void ) = 0;

protected:
    Mat33< T > fR;
    Vec3< T > fT;

private:

}; // class C3DObj

template < typename T >
inline C3DObj< T >::C3DObj( void )
{
	fT = Vec3< T >( 0, 0, 0 );

    // REVISIT - refactor this identity matrix
	fR[ 0 ][ 0 ] = 1.0; fR[ 0 ][ 1 ] = 0.0; fR[ 0 ][ 2 ] = 0.0;
	fR[ 1 ][ 0 ] = 0.0; fR[ 1 ][ 1 ] = 1.0; fR[ 1 ][ 2 ] = 0.0;
	fR[ 2 ][ 0 ] = 0.0; fR[ 2 ][ 1 ] = 0.0; fR[ 2 ][ 2 ] = 1.0;
}

} // namespace ChaoVis

#endif // _C3DOBJ_H_
