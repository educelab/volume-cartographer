// UVecMatOperations.h
// Chao Du 2015 Mar
#pragma once

#include "mathUtils.h"
#include "CVectorN.h"
#include "CMatrixMN.h"

namespace ChaoVis {

// O. individual operations
template < typename T >
inline CMatrixMN< T > Zero( int nRows, int nCols )
{
    CMatrixMN< T > aResult( nRows, nCols );
    for ( int i = 0; i < nRows; ++i ) {
        for ( int j = 0; j < nCols; ++j ) {
            aResult.Set( i, j, T( 0 ) );
        }
    }
    return aResult;
}

template < typename T >
inline CMatrixMN< T > Eye( int nSize )
{
    CMatrixMN< T > aResult = Zero< T >( nSize, nSize );
    for ( int i = 0; i < nSize; ++i ) {
        aResult.Set( i, i, T( 1 ) );
    }
    return aResult;
}

template < typename T >
inline CMatrixMN< T > Diagnal( const CVectorN< T > &nVec )
{
    int aSize = nVec.GetLength();
    CMatrixMN< T > aResult( aSize, aSize );
    for ( int i = 0; i < aSize; ++i ) {
        aResult.Set( i, i, nVec.Get( i ) );
    }
    return aResult;
}

template < typename T >
inline void SetColumn( CMatrixMN< T > &nMat,
                       const CVectorN< T > &nVec,
                       int nColumnIndex )
{
    int aNumRows = nMat.GetRows();
    assert( aNumRows == nVec.GetLength() &&
            nColumnIndex < nMat.GetCols() );

    for ( int i = 0; i < aNumRows; ++i ) {
        nMat.Set( i, nColumnIndex, nVec.Get( aNumRows ) );
    }
}

template < typename T >
inline T CalcDeterminant2x2( const CMatrixMN< T > &nMat )
{
    assert( nMat.GetRows() == 2 &&
            nMat.GetCols() == 2 );
    return( nMat( 0, 0 ) * nMat( 1, 1 ) - nMat( 0, 1 ) * nMat( 1, 0 ) );
}

template < typename T >
inline T CalcDeterminant3x3( const CMatrixMN< T > &nMat )
{
    assert( nMat.GetRows() == 3 &&
            nMat.GetCols() == 3 );
	return( nMat( 0, 0 ) * nMat( 1, 1 ) * nMat( 2, 2 ) - nMat( 0, 0 ) * nMat( 2, 1 ) * nMat( 1, 2 ) +
			nMat( 0, 1 ) * nMat( 2, 0 ) * nMat( 1, 2 ) - nMat( 0, 1 ) * nMat( 1, 0 ) * nMat( 2, 2 ) + 
			nMat( 0, 2 ) * nMat( 1, 0 ) * nMat( 2, 1 ) - nMat( 0, 2 ) * nMat( 1, 1 ) * nMat( 2, 0 ) );
}

template < typename T >
inline T CalcDeterminantNxN( const CMatrixMN< T > &nMat )
{
    int aSize = nMat.GetRows();
    assert( aSize > 0 &&
            aSize == nMat.GetCols() );

    // recursively calculate the determinant of the matrix
    T aDetValue = T( 0 );
    if ( aSize == 1 ) { // scalar

        aDetValue = nMat( 0, 0 );

    } else if ( aSize == 2 ) { // 2x2

        aDetValue = nMat( 0, 0 ) * nMat( 1, 1 ) - nMat( 0, 1 ) * nMat( 1, 0 );

    } else { // matrix larger than 2x2

        // iterate through all sub-matrices
        for ( int i = 0; i < aSize; ++i ) {

            // create a sub-matrix
            // REVISIT - could utilize get-vector operations
            CMatrixMN< T > aSubMat( aSize - 1, aSize - 1 );
            int aColIndex;

            for ( int j = 1; j < aSize; ++j ) { // start from the second row

                aColIndex = 0;

                for ( int k = 0; k < aSize; ++k ) {

                    // skip one colume
                    if ( k == i ) {
                        continue;
                    }

                    aSubMat.Set( j - 1, aColIndex, nMat.Get( j, k ) );
                    ++aColIndex;

                } // for k

            } // for j

            aDetValue += pow( -1, i ) * nMat.Get( 0, i ) * CalcDeterminantNxN( aSubMat );

        } // for i

    }
    return aDetValue;
}

// Create sub matrix by eliminating the specified row and column
template < typename T >
inline CMatrixMN< T > GetSubMatrix( const CMatrixMN< T > &nMat,
                                    int nRow,
                                    int nCol )
{
    int aRows = nMat.GetRows();
    int aCols = nMat.GetCols();

    int aRowCount = 0;
    int aColCount = 0;

    CMatrixMN< T > aResult( aRows - 1, aCols - 1 );

    for ( int i = 0; i < aRows; ++i ) {

        if ( i == nRow ) { // skip the row
            continue;
        }

        aColCount = 0;

        for ( int j = 0; j < aCols; ++j ) {

            if ( j == nCol ) { // skip the column
                continue;
            }

            aResult( aRowCount, aColCount ) = nMat( i, j );

            aColCount++;

        } // for j

        aRowCount++;

    } // for i

    return aResult;
}

// Inverse
template < typename T >
inline CMatrixMN< T > Inverse( const CMatrixMN< T > &nMat )
{
    // REVISIT - this is a simple inverse routine by finding the adjugate/adjoint matrix; 
    //           a general nxn matrix can be inverted using methods such as Gaussian elimination or LU decomposition.
    int aDim = nMat.GetCols();
    assert( aDim == nMat.GetRows() );

    CMatrixMN< T > aResult( aDim, aDim );
    double aDet = CalcDeterminantNxN< T >( nMat );
    if ( fabs( aDet ) < 1e-6 ) {
        printf( "WARNING: matrix is close to singular. Return zero matrix.\n" );
        return aResult;
    }

    for ( int i = 0; i < aDim; ++i ) {

        for ( int j = 0; j < aDim; ++j ) {

            // create the sub-matrix
            CMatrixMN< double > aAdjointMat( aDim - 1, aDim - 1 );
            aAdjointMat = GetSubMatrix< double >( nMat, j, i ); // REVISIT - notice to transpose the matrix

            double aAdjointElement = pow( -1, i + j ) * CalcDeterminantNxN< double >( aAdjointMat );

            aResult( i, j ) = aAdjointElement / aDet;

        } // for j

    } // for i

    return aResult;
}

// I. vector/vector operations

// operator+
template < typename T >
inline CVectorN< T > operator+( const CVectorN< T > &nVec1,
                                const CVectorN< T > &nVec2 )
{
    int aLen = nVec1.GetLength();
    assert( aLen == nVec2.GetLength() );

    CVectorN< T > aResult( nVec1 );
    for ( int i = 0; i < aLen; ++i ) {
        aResult.Set( i, aResult.Get( i ) + nVec2.Get( i ) );
    }
    return aResult;
}

// operator-
template < typename T >
inline CVectorN< T > operator-( const CVectorN< T > &nVec1,
                                const CVectorN< T > &nVec2 )
{
    int aLen = nVec1.GetLength();
    assert( aLen == nVec2.GetLength() );

    CVectorN< T > aResult( nVec1 );
    for ( int i = 0; i < aLen; ++i ) {
        aResult.Set( i, aResult.Get( i ) - nVec2.Get( i ) );
    }
    return aResult;
}

// operator*, scalar * vector
template < typename T >
inline CVectorN< T > operator*( T nScalar,
                                const CVectorN< T > &nVec )
{
    CVectorN< T > aResult( nVec.GetLength() );
    for ( int i = 0; i < nVec.GetLength(); ++i ) {
        aResult[ i ] = nVec[ i ] * nScalar;
    }
    return aResult;
}

// operator*, vector * scalar
template < typename T >
inline CVectorN< T > operator*( const CVectorN< T > &nVec,
                                T nScalar )
{
    CVectorN< T > aResult( nVec.GetLength() );
    for ( int i = 0; i < nVec.GetLength(); ++i ) {
        aResult[ i ] = nVec[ i ] * nScalar;
    }
    return aResult;
}

// operator*, scalar * matrix
template < typename T >
inline CMatrixMN< T > operator*( T nScalar,
                                const CMatrixMN< T > &nMat )
{
    int aRows = nMat.GetRows();
    int aCols = nMat.GetCols();
    CMatrixMN< T > aResult( aRows, aCols );
    for ( int i = 0; i < aRows; ++i ) {
        for ( int j = 0; j < aCols; ++j ) {
            // REVISIT - IMPROVE - use STL memory block operation
            aResult( i, j ) = nMat( i, j ) * nScalar;
        }
    }
    return aResult;
}

// operator*, matrix * scalar
template < typename T >
inline CMatrixMN< T > operator*( const CMatrixMN< T > &nMat,
                                T nScalar )
{
    int aRows = nMat.GetRows();
    int aCols = nMat.GetCols();
    CMatrixMN< T > aResult( aRows, aCols );
    for ( int i = 0; i < aRows; ++i ) {
        for ( int j = 0; j < aCols; ++j ) {
            // REVISIT - IMPROVE - use STL memory block operation
            aResult( i, j ) = nMat( i, j ) * nScalar;
        }
    }
    return aResult;
}

// operator*
// vector products can be treated as matrix multiplication; however, the inner product is treated specially
// we don't have a way to distinguish row vector and column vector (is that necessary?)
template < typename T >
inline CMatrixMN< T > operator*( const CVectorN< T > &nVec1,
                                 const CVectorN< T > &nVec2 )
{
    int aNumRows = nVec1.GetLength();
    int aNumCols = nVec2.GetLength();

    CMatrixMN< T > aResult( aNumRows, aNumCols );
    for ( int i = 0; i < aNumRows; ++i ) {
        for ( int j = 0; j < aNumCols; ++j ) {
            aResult.Set( i, j, nVec1.Get( i ) * nVec2.Get( j ) );
        } // for j
    } // for i
    return aResult;
}

// inner product
template < typename T >
inline T CalcInnerProduct( const CVectorN< T > &nVec1,
                           const CVectorN< T > &nVec2 )
{
    int aLen = nVec1.GetLength();
    assert( aLen == nVec2.GetLength() );

    T aResult = T( 0 );
    for ( int i = 0; i < aLen; ++i ) {
        aResult += nVec1.Get( i ) * nVec2.Get( i );
    }
    return aResult;
}

// II. vector/matrix operations

// matrix * vector, vector is treated as column vector
template < typename T >
inline CVectorN< T > operator*( const CMatrixMN< T > &nMat,
                                const CVectorN< T > &nVec )
{
    int aNumRows = nMat.GetRows();
    int aNumCols = nMat.GetCols();
    assert( aNumCols == nVec.GetLength() );

    CVectorN< T > aResult( aNumRows );
    for ( int i = 0; i < aNumRows; ++i ) {
        T aTmp = T( 0 );
        for ( int j = 0; j < aNumCols; ++j ) {
            aTmp += nMat.Get( i, j ) * nVec.Get( j );
        } // for j
        aResult.Set( i, aTmp );
    } // for i
    return aResult;
}

// vector * matrix, vector is treated as row vector
template < typename T >
inline CVectorN< T > operator*( const CVectorN< T > &nVec,
                                const CMatrixMN< T > &nMat )
{
    int aNumRows = nMat.GetRows();
    int aNumCols = nMat.GetCols();
    assert( nVec.GetLength() == aNumRows );

    CVectorN< T > aResult( aNumCols );
    for ( int i = 0; i < aNumCols; ++i ) {
        T aTmp = T( 0 );
        for ( int j = 0; j < aNumRows; ++j ) {
            aTmp += nVec.Get( j ) * nMat.Get( j, i );
        } // for j
        aResult.Set( i, aTmp );
    } // for i
    return aResult;
}

// III. maxtrix/matrix operations

// operator+
template < typename T >
inline
CMatrixMN< T > operator+( const CMatrixMN< T > &nMat1,
                          const CMatrixMN< T > &nMat2 )
{
    int aNumRows = nMat1.GetRows();
    int aNumCols = nMat1.GetCols();
    assert( aNumRows == nMat2.GetRows() &&
            aNumCols == nMat2.GetCols() );

    CMatrixMN< T > aResult( nMat1 );
    for ( int i = 0; i < aNumRows; ++i ) {
        for ( int j = 0; j < aNumCols; ++j ) {
            aResult.Set( i, j, aResult.Get( i, j ) + nMat2.Get( i, j ) );
        }
    }
    return aResult;
}

// operator-
template < typename T >
inline
CMatrixMN< T > operator-( const CMatrixMN< T > &nMat1,
                          const CMatrixMN< T > &nMat2 )
{
    int aNumRows = nMat1.GetRows();
    int aNumCols = nMat1.GetCols();
    assert( aNumRows == nMat2.GetRows() &&
            aNumCols == nMat2.GetCols() );

    CMatrixMN< T > aResult( nMat1 );
    for ( int i = 0; i < aNumRows; ++i ) {
        for ( int j = 0; j < aNumCols; ++j ) {
            aResult.Set( i, j, aResult.Get( i, j ) - nMat2.Get( i, j ) );
        }
    }
    return aResult;
}

// operator*
template < typename T >
inline
CMatrixMN< T > operator*( const CMatrixMN< T > &nMat1,
                          const CMatrixMN< T > &nMat2 )
{
    int aNumZip = nMat1.GetCols();
    assert( aNumZip == nMat2.GetRows() );

    int aNumRows = nMat1.GetRows();
    int aNumCols = nMat2.GetCols();
    CMatrixMN< T > aResult( aNumRows, aNumCols );
    for ( int i = 0; i < aNumRows; ++i ) {
        for ( int j = 0; j < aNumCols; ++j ) {
            T aTmp = T( 0 );
            for ( int k = 0; k < aNumZip; ++k ) {
                aTmp += nMat1.Get( i, k ) * nMat2.Get( k, j );
            } // for k
            aResult.Set( i, j, aTmp );
        } // for j
    } // for i
    return aResult;
}


// IV. Miscellaneous


// Singular value decomposition: reorder
// given the ouput fo decompose, this routine sorts the singular values, and corresponding columns of u and v, by decreasing magnitude. Also, signs of corresponding columns are flipped so as to maximize the number of positive elements.
template < typename T >
inline void reorder( CMatrixMN< T > &u,
                     CVectorN< T > &w,
                     CMatrixMN< T > &v ) {
    int i, j, k, s, inc = 1;
    int m = u.GetRows();
    int n = u.GetCols();
    double sw;
    CVectorN< T > su( m ), sv( n );

    do {
        inc *= 3;
        inc++;
    } while ( inc <= n );

    // sort. The method is Shell's sort (generalized insert sort).
    // (The work is negligible as compared to that already done in decompose.)
    do {
        inc /= 3;
        for ( i = inc; i < n; ++i ) {
            sw = w[ i ];
            for ( k = 0; k < m; ++k ) {
                su[ k ] = u( k, i );
            } // for k
            for ( k = 0; k < n; ++k ) {
                sv[ k ] = v( k, i );
            } // for k
            j = i;
            while ( w[ j - inc ] < sw ) {
                w[ j ] = w[ j - inc ];
                for ( k = 0; k < m; ++k ) {
                    u( k, j ) = u( k, j - inc );
                } // for k
                for ( k = 0; k < n; ++k ) {
                    v( k, j ) = v( k, j - inc );
                } // for k
                j -= inc;
                if ( j < inc ) {
                    break;
                }
            } // while
            w[ j ] = sw;
            for ( k = 0; k < m; ++k ) {
                u( k, j ) = su[ k ];
            } // for k
            for ( k = 0; k < n; ++k ) {
                v( k, j ) = sv[ k ];
            } // for k
        } // for i
    } while ( inc > 1 );

    // flip signs.
    for ( k = 0; k < n; ++k ) {
        s = 0;
        for ( i = 0; i < m; ++i ) {
            if ( u( i, k ) < 0. ) {
                s++;
            }
        } // for i
        for ( j = 0; j < n; ++j ) {
            if ( v( j, k )  < 0. ) {
                s++;
            }
        } // for j
        if ( s > ( m + n ) / 2 ) {
            for ( i = 0; i < m; ++i ) {
                u( i, k ) = -u( i, k );
            } // for i
            for ( j = 0; j < n; ++j ) {
                v( j, k ) = -v( j, k );
            } // for j
        } // if
    } // for k
}

// Singular value decomposition
// REVISIT - refer to "Numerical Recipes in C" for detailed implementation
// A = UWV^T
// modified by Chao Du 2015 March
template < typename T >
inline void SVD( CMatrixMN< T > &nU,
                 CVectorN< T > &nW,
                 CMatrixMN< T > &nV,
                 const CMatrixMN< T > &nMat )
{
    // implementation of singular value decomposition, M = U W V'

    CMatrixMN< T > aA( nMat );

    int flag, i, its, j, jj, k, l, nm;
    int m = nMat.GetRows();
    int n = nMat.GetCols();
    int dim = m < n ? m : n;
    float anorm, c, f, g, h, s, scale, x, y, z;

    CVectorN< T > aW( m > n ? m : n );
    CVectorN< T > rv1( n );

    g = scale = anorm = 0.0;

    // Householder reduction to diagonal form
    for ( i = 0; i < n; ++i ) {

        l = i + 2;
        rv1[ i ] = scale * g;
        g = s = scale = 0.0;

        if ( i < m ) {

            for ( k = i; k < m; ++k ) {
                scale += fabs( aA( k, i ) );
            }

            if ( fabs( scale ) > 1e-6 ) {

                for ( k = i; k < m; ++k ) {
                    aA( k, i ) /= scale;
                    s += aA( k, i ) * aA( k, i );
                }

                f = aA( i, i );
                g = -SIGN( sqrt( s ), f );
                h = f * g - s;
                aA( i, i ) = f - g;
 
                for ( j = l - 1; j < n; ++j ) {
 
                    for ( s = 0.0, k = i; k < m; ++k ) {
                        s += aA( k, i ) * aA( k, j );
                    } // for k
 
                    f = s / h;
 
                    for ( k = i; k < m; k++ ) {
                        aA( k, j ) += f * aA( k, i );
                    } // for k
 
                } // for j
 
                for ( k = i; k < m; ++k ) {
                    aA( k, i ) *= scale;
                } // for k

            } // if

        } // if

        aW[ i ] = scale * g;
        g = s = scale = 0.0;
        if ( i + 1 <= m && i != ( n - 1 ) ) {

            for ( k = l - 1; k < n; ++k ) {
                scale += fabs( aA( i, k ) );
            }

            if ( fabs( scale ) > 1e-6 ) {

                for ( k = l - 1; k < n; ++k ) {
                    aA( i, k ) /= scale;
                    s += aA( i, k ) * aA( i, k );
                } // for k

                f = aA( i, l - 1 );
                g = -SIGN( sqrt( s ), f );
                h = f * g - s;
                aA( i, l - 1 ) = f - g;

                for ( k = l - 1; k < n; ++k ) {
                    rv1[ k ] = aA( i, k ) / h;
                } // for k

                for ( j = l - 1; j < m; ++j ) {

                    for ( s = 0.0, k = l - 1; k < n; ++k ) {
                        s += aA( j, k ) * aA( i, k );
                    } // for k

                    for ( k = l - 1; k < n; ++k ) {
                        aA( j, k ) += s * rv1[ k ];
                    } // for k

                } // for j

                for ( k = l - 1; k < n; ++k ) {
                    aA( i, k ) *= scale;
                } // for k

            } // if

        } // if

        anorm = MAX( anorm, ( fabs( aW[ i ]) + fabs( rv1[ i ] ) ) );

    } // for i

    // Accumulation of right-hand transformations.
    for ( i = n - 1; i >= 0; --i ) {

        if ( i < n - 1 ) {

            if ( fabs( g ) > 1e-6 ) {

                // Double division to avoid possible underflow.
                for ( j = l; j < n; ++j ) {
                    nV( j, i ) = ( aA( i, j ) / aA( i, l ) ) / g;
                } // for j

                for ( j = l; j < n; ++j ) {

                    for ( s = 0.0, k = l; k < n; ++k ) {
                        s += aA( i, k ) * nV( k, j );
                    } // for k

                    for ( k = l; k < n; ++k ) {
                        nV( k, j ) += s * nV( k, i );
                    } // for k

                } // for j

            } // if

            for ( j = l; j < n; ++j ) {
                nV( i, j ) = 0.0;
                nV( j, i ) = 0.0;
            } // for j

        } // if

        nV( i, i ) = 1.0;
        g = rv1[ i ];
        l = i;

    } // for i

    // Accumulation of left-hand transformations.
    for ( i = dim - 1; i >= 0; --i ) {

        l = i + 1;
        g = aW[ i ];

        for ( j = l; j < n; ++j ) {
            aA( i, j ) = 0.0;
        } // for j

        if ( fabs( g ) > 1e-6 ) {

            g = 1.0 / g;

            for ( j = l; j < n; ++j ) {

                for ( s = 0.0, k = l; k < m; ++k ) {
                    s += aA( k, i ) * aA( k, j );
                } // for k

                f = ( s / aA( i, i ) ) * g;

                for ( k = i; k < m; ++k ) {
                    aA( k, j ) += f * aA( k, i );
                } // for k

            } // for j

            for ( j = i; j < m; ++j ) {
                aA( j, i ) *= g;
            } // for j

        } else {

            for ( j = i; j < m; ++j ) {
                aA( j, i ) = 0.0;
            } // for j

        } // if

        ++aA( i, i );

    } // for i

    // Diagonalization of the bidiagonal form: Loop over singular values, and over allowed iterations.
    for ( k = n - 1; k >= 0; --k ) {

        for ( its = 1; its <= 30; ++its ) {

            flag = 1;

            // Test for splitting. Note that rv1[ 0 ] is always zero.
            for ( l = k; l >= 0; --l ) {

                nm = l - 1;

                if ( l == 0 || fabs( ( float )( fabs( rv1[ l ] ) + anorm ) - anorm ) < 1e-6 ) {
                    flag = 0;
                    break;
                } // if

                if ( fabs( ( float )( fabs( aW[ nm ] ) + anorm ) - anorm ) < 1e-6 ) {
                    break;
                }

            } // for l

            // Cancellation of rv1[ l ], if l > 1.
            if ( flag ) {

                c = 0.0;
                s = 1.0;

                for ( i = l; i <= k; ++i ) {
                    f = s * rv1[ i ];
                    rv1[ i ] = c * rv1[ i ];

                    if ( fabs( ( float )( fabs( f ) + anorm ) - anorm ) < 1e-6 ) {
                        break;
                    }
                    g = aW[ i ];
                    h = pythag< float >( f, g );
                    aW[ i ] = h;
                    h = 1.0 / h;
                    c = g * h;
                    s = -f * h;
                    for ( j = 0; j < m; ++j ) {
                        y = aA( j, nm );
                        z = aA( j, i );
                        aA( j, nm ) = y * c + z * s;
                        aA( j, i ) = z * c - y * s;
                    } // for j

                } // for i

            } // if

            z = aW[ k ];

            // Convergence. Singular value is made nonnegative.
            if ( l == k ) {
                if ( z < 0.0 ) {
                    aW[ k ] = -z;
                    for ( j = 0; j < n; ++j ) {
                        nV( j, k ) = -nV( j, k );
                    } // for j
                } // if
                break;
            } // if

            if ( its == 30 ) {
                printf( "ERROR: no convergence in 30 svdcmp iterations" );
                exit( 1 );
            } // if

            // Shift from bottom 2-by-2 minor.
            x = aW[ l ];
            nm = k - 1;
            y = aW[ nm ];
            g = rv1[ nm ];
            h = rv1[ k ];
            f = ( ( y - z ) * ( y + z ) + ( g - h ) * ( g + h ) ) / ( 2.0 * h * y );
            g = pythag< float >( f, 1.0 );
            f = ( ( x - z ) * ( x + z ) + h * ( ( y / ( f + SIGN( g, f ) ) ) - h ) ) / x;
            c = s = 1.0;

            // Next QR transformation:
            for ( j = l; j <= nm; ++j ) {

                i = j + 1;
                g = rv1[ i ];
                y = aW[ i ];
                h = s * g;
                g = c * g;
                z = pythag< float >( f, h );
                rv1[ j ] = z;
                c = f / z;
                s = h / z;
                f = x * c + g * s;
                g = g * c - x * s;
                h = y * s;
                y *= c;

                for ( jj = 0; jj < n; ++jj ) {
                    x = nV( jj, j );
                    z = nV( jj, i );
                    nV( jj, j ) = x * c + z * s;
                    nV( jj, i ) = z * c - x * s;
                } // for jj

                z = pythag< float >( f, h );
                aW[ j ] = z;

                // Rotation can be arbitrary if z = 0.
                if ( fabs( z ) > 1e-6 ) {
                    z = 1.0 / z;
                    c = f * z;
                    s = h * z;
                } // if

                f = c * g + s * y;
                x = c * y - s * g;

                for ( jj = 0; jj < m; ++jj ) {
                    y = aA( jj, j );
                    z = aA( jj, i );
                    aA( jj, j ) = y * c + z * s;
                    aA( jj, i ) = z * c - y * s;
                } // for jj

            } // for j

            rv1[ l ] = 0.0;
            rv1[ k ] = f;
            aW[ k ] = x;

        } // for its

    } // for k

    reorder( aA, aW, nV );

    nW = aW;
    printf( "U:\n");
    aA.Dump();

    // copy back to U
    for ( int i = 0; i < aA.GetRows(); ++i ) {
        for ( int j = 0; j < aA.GetCols(); ++j ) {
            nU( i, j ) = aA( i, j );
        } // for j
    } // for i
}

template < typename T >
inline void SVD3x3( CMatrixMN< T > &nU,
                    CMatrixMN< T > &nW,
                    CMatrixMN< T > &nV,
                    const CMatrixMN< T > &nMat )
{
    // REVISIT - FILL ME HERE
    // implementation of singular value decomposition, M = U W V'
    // ...
}

template < typename T >
inline void SVD4x4( CMatrixMN< T > &nU,
                    CMatrixMN< T > &nW,
                    CMatrixMN< T > &nV,
                    const CMatrixMN< T > &nMat )
{
    // REVISIT - FILL ME HERE
    // implementation of singular value decomposition, M = U W V'
    // ...
}

// QR decomposition
template < typename T >
inline void QR( CMatrixMN< T > &nQ,
                CMatrixMN< T > &nR,
                const CMatrixMN< T > &nMat )
{
    // REVISIT - FILL ME HERE
    // implementation of QR decomposition, M = Q R
}

template < typename T >
inline void QR3x3( CMatrixMN< T > &nQ,
                   CMatrixMN< T > &nR,
                   const CMatrixMN< T > &nMat )
{
    // REVISIT - FILL ME HERE
    // implementation of QR decomposition, M = Q R
}

// Chelosky decomposition
template < typename T >
inline void Chelosky( CMatrixMN< T > &nL,
                      const CMatrixMN< T > &nMat )
{
    // REVISIT - FILL ME HERE
    // implementation of Chelosky decomposition, M = L L'
}

// Thomas tri-diagonal matrix
// en.wikipedia.org/wiki/Tridiagonal_matrix_algorithm
template < typename T >
inline void ThomasTriDiagonal( const CMatrixMN< T > &nA,
                               const CVectorN< T > &nB,
                               CVectorN< T > &nResult )
{
    int aSize = nB.GetLength();
    assert( aSize == nA.GetRows() && aSize == nA.GetCols() );

    CVectorN< T > aC( aSize ), aD( aSize );
    aC[ 0 ] = nA( 0, 1 ) / nA( 0, 0 );
    for ( int i = 1; i < aSize - 1; ++i ) {
        aC[ i ] = nA( i, i + 1 ) / ( nA( i, i ) - nA( i, i - 1 ) * aC[ i - 1 ] );
    }
    aD[ 0 ] = nB[ 0 ] / nA( 0, 0 );
    for ( int i = 1; i < aSize; ++i ) {
        aD[ i ] = ( nB[ i ] - nA( i, i - 1 ) * aD[ i - 1 ] ) / ( nA( i, i ) - nA( i, i - 1 ) * aC[ i - 1 ] );
    }
    nResult[ aSize - 1 ] = aD[ aSize - 1 ];
    for ( int i = aSize - 2; i >= 0; --i ) {
        nResult[ i ] = aD[ i ] - aC[ i ] * nResult[ i + 1 ];
    }
}


} // namespace ChaoVis
