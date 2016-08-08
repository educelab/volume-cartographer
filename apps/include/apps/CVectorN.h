// CVectorN.h
// Chao Du 2015 Jan
#ifndef _CVECTORN_H_
#define _CVECTORN_H_


#include "apps/HBase.h"
#include <string.h>
#include <math.h>
#include <assert.h>
#include <stdio.h>

namespace ChaoVis {

template < typename T >
class CVectorN {

public:
    CVectorN( void );
    CVectorN( int nLen );
    CVectorN( int nLen, const T *nData );
	CVectorN( const CVectorN &nCpy );
	~CVectorN( void );

	T& operator[]( int nIndex );
    T operator[]( int nIndex ) const;
    CVectorN& operator=( const CVectorN &nCpy ); // REVISIT - remember, return reference, not a copy!
    CVectorN& operator+=( const CVectorN &nRHS );

	int GetLength( void ) const;

    T Get( int nIndex ) const;
    void Set( int nIndex, T nValue );

    const T* GetDataConst( void ) const;
	T* GetData( void );

    double CalcNormOne( void );
    double CalcNormTwo( void );
    double CalcNormInfinity( void );

    void Dump( void ) const; // REVISIT - output function, should overload stream operator << as well

protected:

private:
    T *fData;
    int fLen;

}; // class CVectorN

// Constructor
template < typename T >
inline CVectorN< T >::CVectorN( void ) :
    fLen( 0 ),
    fData( NULL )
{
}

// Constructor
template < typename T >
inline CVectorN< T >::CVectorN( int nLen ) :
    fLen( nLen ),
    fData( NULL )
{
    // REVISIT - FILL ME HERE fill in the data
    // REVISIT - IMPROVE - this is far less efficient! change this to vector/batch operation!!!
    fData = new T[ nLen ];
    memset( fData, 0, nLen * sizeof( T ) );
}

// Constructor
template < typename T >
inline CVectorN< T >::CVectorN( int nLen, const T *nData ) :
    fLen( nLen ),
    fData( NULL )
{
    // REVISIT - FILL ME HERE fill in the data
    // REVISIT - IMPROVE - this is far less efficient! change this to vector/batch operation!!!
    fData = new T[ nLen ];
    memcpy( fData, nData, nLen * sizeof( T ) );
}

// Copy constructor
template < typename T >
inline CVectorN< T >::CVectorN( const CVectorN &nCpy ) :
    fLen( nCpy.GetLength() ),
    fData( NULL )
{
    fData = new T[ fLen ];
    // REVISIT - why do we get const pointer then cast it away?
    memcpy( fData, static_cast< const void * >( nCpy.GetDataConst() ), fLen * sizeof( T ) );
}

// Destructor
template < typename T >
inline CVectorN< T >::~CVectorN( void )
{
    deleteNULL( fData, true ); // delete array
}

// operator[]
template < typename T >
inline T& CVectorN< T >::operator[]( int nIndex ) { assert( nIndex >= 0 && nIndex < fLen ); return *( fData + nIndex ); }

// operator[]
template < typename T >
inline T CVectorN< T >::operator[]( int nIndex ) const { assert( nIndex >= 0 && nIndex < fLen ); return *( fData + nIndex ); }

// operator=
template < typename T >
inline CVectorN< T >& CVectorN< T >::operator=( const CVectorN &nCpy )
{
    if ( fData == NULL ) {
        fLen = nCpy.GetLength();
        fData = new T[ fLen ];
    }
    assert( fLen == nCpy.GetLength() ); // assert here because we need to make sure the lengths agree if vector is already allocated
    // REVISIT - why do we get const pointer then cast it away?
    memcpy( fData, static_cast< const void * >( nCpy.GetDataConst() ), fLen * sizeof( T ) );
}

// operator+=
template < typename T >
inline CVectorN< T >& CVectorN< T >::operator+=( const CVectorN &nRHS )
{
    assert( nRHS.fLen == fLen );
    for ( int i = 0; i < fLen; ++i ) {
        fData[ i ] += nRHS.fData[ i ];
    }
    return *this;
}

template < typename T >
inline int CVectorN< T >::GetLength( void ) const { return fLen; }

template < typename T >
inline T CVectorN< T >::Get( int nIndex ) const { return *( fData + nIndex ); }
template < typename T >
inline void CVectorN< T >::Set( int nIndex, T nValue ) { *( fData + nIndex ) = nValue; }

template < typename T >
inline const T* CVectorN< T >::GetDataConst( void ) const { return fData; }
template < typename T >
inline T* CVectorN< T >::GetData( void ) { return fData; }

template < typename T >
inline double CVectorN< T >::CalcNormOne( void )
{
    // REVISIT - IMPROVE - this is inefficient, consider vector operation
    double aResult = 0.0;
    for ( int i = 0; i < fLen; ++i ) {
        aResult += *( fData + i );
    }
    return aResult;
}

template < typename T >
inline double CVectorN< T >::CalcNormTwo( void )
{
    // REVISIT - IMPROVE - this is inefficient, consider vector operation
    double aResult = 0.0;
    for ( int i = 0; i < fLen; ++i ) {
        aResult += ( *( fData + i ) * *( fData + i ) );
    }
    return sqrt( aResult );
}

template < typename T >
inline double CVectorN< T >::CalcNormInfinity( void )
{
    // REVISIT - IMPROVE - this is inefficient, consider vector operation
    double aResult = 0.0;
    for ( int i = 0; i < fLen; ++i ) {
        if ( aResult < fabs( *( fData + i ) ) ) {
            aResult = fabs( *( fData + i ) );
        }
    }
    return aResult;
}

template < typename T >
inline void CVectorN< T >::Dump( void ) const
{
    for ( int i = 0; i < fLen; ++i ) {
        printf( "%lf ", fData[ i ] );
    }
    printf( "\n" );
}

} // namespace ChaoVis

#endif // _CVECTORN_H_
