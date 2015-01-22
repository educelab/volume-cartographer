// HBase.h
// Chao Du 2014 Dec
#ifndef _HBASE_H_
#define _HBASE_H_

#include <stdlib.h>

namespace ChaoVis {

#define USHORT_SIZE 65536

template < typename T >
static void deleteNULL( T* &nPtr )
{
    if ( nPtr != NULL ) {
        delete nPtr;
        nPtr = NULL;
    }
}

template < typename T >
static void swap( T &nVal1, T &nVal2 )
{
    T aTmp = nVal1;
    nVal1 = nVal2;
    nVal2 = aTmp;
}

} // namespace ChaoVis

#endif // _HBASE_H_
