// CVec.h
// Chao Du Sept. 2014
#ifndef _CVEC_H_
#define _CVEC_H_

namespace ChaoVis {

template< size_t s, typename T >
class CVec {
	
public:
	T& operator[]( int i ) { return fData[ i ]; }
	const T operator[]( int i ) const { return fData[ i ]; }

private:
	T fData[s];

}; // class CVec

typedef CVec< 2, char   > CVec2c;
typedef CVec< 2, int    > CVec2i;
typedef CVec< 2, float  > CVec2f;
typedef CVec< 2, double > CVec2d;

typedef CVec< 3, char   > CVec3c;
typedef CVec< 3, int    > CVec3i;
typedef CVec< 3, float  > CVec3f;
typedef CVec< 3, double > CVec3d;

} // namespace ChaoVis

#endif // _CVEC_H_
