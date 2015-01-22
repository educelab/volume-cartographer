// C3DObj.h
// Chao Du 2014 Dec
#ifndef _C3DOBJ_H_
#define _C3DOBJ_H_

#include "mathUtils.h"

namespace ChaoVis {

template < typename T >
class C3DObj {

public:
    const Mat33< T >& GetRotation( void ) const { return fR; }
    void SetRotation( const Mat33< T > &nR ) { fR = nR; }
    const Vec3< T >& GetTranslation( void ) const { return fT; }
    void SetTranslation( const Vec3< T > &nT ) { fT = nT; }

    virtual void Draw( void ) = 0;

protected:
    Mat33< T > fR;
    Vec3< T > fT;

private:

}; // class C3DObj

} // namespace ChaoVis

#endif // _C3DOBJ_H_
