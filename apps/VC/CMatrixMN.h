// CMatrixMN.h
// Chao Du 2015 Jan
// mxn matrix
#pragma once

#include "CVectorN.h"
#include "HBase.h"
#include "MathUtils.h"

#include <stdio.h>
#include <string.h>

namespace ChaoVis
{

template <typename T>
class CMatrixMN
{

public:
    CMatrixMN(void);
    CMatrixMN(int nRows, int nCols);
    CMatrixMN(const CMatrixMN& nCpy);
    ~CMatrixMN(void);

    CMatrixMN& operator=(const CMatrixMN<T>& nCpy);  // REVISIT - remember,
                                                     // return reference, not a
                                                     // copy!
    T& operator()(int nRow, int nCol);
    T operator()(int nRow, int nCol) const;
    CMatrixMN& operator+=(const CMatrixMN<T>& nRHS);

    int GetRows(void) const { return fRows; }
    int GetCols(void) const { return fCols; }
    Vec2<int> GetDim(void) { return Vec2<int>(fRows, fCols); }

    // Set a single value
    void Set(int nRow, int nCol, T nValue)
    {
        *(fData + nRow * fCols + nCol) = nValue;
    }
    // Get a single value
    T Get(int nRow, int nCol) const { return (*(fData + nRow * fCols + nCol)); }
    // Fill a sub matrix with a single value
    void Fill(int nLTRow, int nLTCol, int nBRRow, int nBRCol, T nValue)
    {
        // REVISIT - IMPROVE - this is far far far less efficient; use
        // vector/batch operation
        //           for example, std::copy
        for (int i = nLTRow; i < nBRRow; ++i) {
            for (int j = nLTCol; j < nBRCol; ++j) {
                *(fData + i * fCols + j) = nValue;
            }
        }
    }
    // Fill a sub matrix with a matrix
    void Fill(
        int nLTRow,
        int nLTCol,
        int nBRRow,
        int nBRCol,
        const CMatrixMN& nSubMatrix)
    {
        // REVISIT - IMPROVE - this is far far far less efficient; use
        // vector/batch operation
        //           for example, std::copy
        for (int i = nLTRow; i < nBRRow; ++i) {
            for (int j = nLTCol; j < nBRCol; ++j) {
                *(fData + i * fCols + j) =
                    nSubMatrix.Get(i - nLTRow, j - nLTCol);
            }
        }
    }
    // Get sub matrix
    // REVISIT - FILL ME HERE

    const T* GetDataConst(void) const { return fData; }
    T* GetData(void) { return fData; }

    double CalcNormFrobenius(void)
    {
        // REVISIT - IMPROVE - this is inefficient, consider vector operation
        double aResult = 0.0;
        for (int i = 0; i < fRows; ++i) {
            for (int j = 0; j < fCols; ++j) {
                aResult +=
                    (*(fData + i * fCols + j) * *(fData + i * fCols + j));
            }
        }
        return sqrt(aResult);
    }

    void TransposeInPlace(void);
    CMatrixMN<T> Transpose(void);

    CVectorN<T> GetCol(int nColIndex);
    CVectorN<T> GetRow(int nRowIndex);

    void Dump(void) const;  // REVISIT - output function, should overload stream
                            // operator << as well

protected:
private:
    T* fData;
    int fRows;
    int fCols;

};  // class CMatrixMN

// Constructor
template <typename T>
inline CMatrixMN<T>::CMatrixMN(void) : fRows(0), fCols(0), fData(NULL)
{
}

// Constructor
template <typename T>
inline CMatrixMN<T>::CMatrixMN(int nRows, int nCols)
    : fData(NULL), fRows(nRows), fCols(nCols)
{
    if (nRows * nCols !=
        0) {  // make sure fData is NULL when dimension is not set
        fData = new T[fRows * fCols];
    }
}

// Copy constructor
template <typename T>
inline CMatrixMN<T>::CMatrixMN(const CMatrixMN<T>& nCpy)
    : fData(NULL), fRows(nCpy.fRows), fCols(nCpy.fCols)
{
    if (fRows * fCols !=
        0) {  // make sure fData is NULL when dimension is not set
        fData = new T[fRows * fCols];
        memcpy(fData, nCpy.GetDataConst(), sizeof(T) * fRows * fCols);
    }
}

// Destructor
template <typename T>
inline CMatrixMN<T>::~CMatrixMN(void)
{
    deleteNULL(fData, true);  // delete array
}

// operator=
template <typename T>
inline CMatrixMN<T>& CMatrixMN<T>::operator=(const CMatrixMN<T>& nCpy)
{
    if (fData == NULL) {
        fRows = nCpy.fRows;
        fCols = nCpy.fCols;
        fData = new T[fRows * fCols];
    } else {
        assert(fRows == nCpy.fRows && fCols == nCpy.fCols);
    }
    memcpy(fData, nCpy.GetDataConst(), sizeof(T) * fRows * fCols);
}

// operator()
template <typename T>
inline T& CMatrixMN<T>::operator()(int nRow, int nCol)
{
    return *(fData + nRow * fCols + nCol);
}

// operator()
template <typename T>
inline T CMatrixMN<T>::operator()(int nRow, int nCol) const
{
    return *(fData + nRow * fCols + nCol);
}

// operator+=
template <typename T>
inline CMatrixMN<T>& CMatrixMN<T>::operator+=(const CMatrixMN<T>& nRHS)
{
    int aRows = nRHS.GetRows();
    int aCols = nRHS.GetCols();
    assert(aRows == fRows && aCols == fCols);

    for (int i = 0; i < aRows; ++i) {
        for (int j = 0; j < aCols; ++i) {
            *(fData + i * fCols + j) += nRHS(i, j);
        }
    }
}

// Transpose the matrix in place - only works for square matrices
template <typename T>
void CMatrixMN<T>::TransposeInPlace(void)
{
    // assert( fCols == fRows );
    for (int i = 0; i < fRows; ++i) {
        for (int j = 0; j < fCols; ++j) {
            Swap(*(fData + i * fCols + j), *(fData + j * fCols + i));
        }
    }
}

// Transpose the matrix
template <typename T>
CMatrixMN<T> CMatrixMN<T>::Transpose(void)
{
    CMatrixMN<T> aResult(fCols, fRows);
    for (int i = 0; i < fRows; ++i) {
        for (int j = 0; j < fCols; ++j) {
            aResult.Set(j, i, *(fData + i * fCols + j));
        }
    }
    return aResult;
}

// Get column of the matrix
template <typename T>
CVectorN<T> CMatrixMN<T>::GetCol(int nColIndex)
{
    // REVISIT - FILL ME HERE
    CVectorN<T> aResult(fRows);
    for (int i = 0; i < fRows; ++i) {
        aResult.Set(i, Get(i, nColIndex));
    }
    return aResult;
}

// Get row of the matrix
template <typename T>
CVectorN<T> CMatrixMN<T>::GetRow(int nRowIndex)
{
    // REVISIT - FILL ME HERE
    CVectorN<T> aResult(fCols);
    for (int i = 0; i < fCols; ++i) {
        aResult.Set(i, Get(nRowIndex, i));
    }
    return aResult;
}

// Dump
template <typename T>
void CMatrixMN<T>::Dump(void) const
{
    for (int i = 0; i < fRows; ++i) {
        for (int j = 0; j < fCols; ++j) {
            printf("%lf ", fData[i * fCols + j]);
        }  // for j
        printf("\n");
    }  // for i
}

}  // namespace ChaoVis
