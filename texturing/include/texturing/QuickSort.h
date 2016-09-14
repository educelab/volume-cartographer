// QuickSort.h
// Chao Du 20141206
#pragma once

namespace ChaoVis {
template < typename T >
void QuickSort( T *nArray,
                int nHead,
                int nTail,
                bool ( *IsLess )( const T &nV1, const T &nV2 ) )
{
    T aPivot = nArray[ nTail ];	// can use different pivot choice
    T aTmp;

    int i = nHead;
    int j = nTail;

    while ( i < j ) {
        while ( IsLess( nArray[ i ], aPivot ) ) {
            ++i;
        }
        while ( IsLess( aPivot, nArray[ j ] ) ) {
            --j;
        }

        // swap
        if ( i <= j ) {
            aTmp = nArray[ i ];
            nArray[ i ] = nArray[ j ];
            nArray[ j ] = aTmp;
            ++i;
            --j;
        }
    }

    // do quick sort recursively
    if ( j > nHead ) {
        QuickSort< T >( nArray, nHead, j, IsLess );
    }
    if ( i < nTail ) {
        QuickSort< T >( nArray, i, nTail, IsLess );
    }
}

} // namespace ChaoVis
