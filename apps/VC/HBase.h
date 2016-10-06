// HBase.h
// Chao Du 2014 Dec
#pragma once

#include <math.h>
#include <stdlib.h>

namespace ChaoVis
{

#define USHORT_SIZE 65536

template <typename T>
inline static void deleteNULL(T*& nPtr, bool nIsArray = false)
{
    if (nPtr != NULL) {
        if (nIsArray) {
            delete[] nPtr;
        } else {
            delete nPtr;
        }
        nPtr = NULL;
    }
}

template <typename T>
inline static void swap(T& nVal1, T& nVal2)
{
    T aTmp = nVal1;
    nVal1 = nVal2;
    nVal2 = aTmp;
}

inline bool IsPowerOfTwo(int n) { return ((n & (n - 1)) == 0); }

inline double LinearImpactFunc(
    double nStartVal, double nCurrentPos, double nRange)
{
    return (nStartVal * (1.0 - nCurrentPos / nRange));
}

inline double ExponentialImpactFunc(
    double nStartVal, double nCurrentPos, double nRange)
{
    return (
        nStartVal /
        pow(M_E,
            nCurrentPos / nRange));  // REVISIT - use lookup table to speed up
}

inline double SigmoidImpactFunc(
    double nStartVal, double nCurrentPos, double nRange)
{
    return (
        nStartVal *
        (1.0 - 1.0 / (1.0 + pow(M_E, -(0.5 + nCurrentPos / nRange) * 1000.0))));
}

inline double GaussianImpactFunc(
    double nStartVal, double nCurrentPos, double nRange)
{
    return (nStartVal * (pow(M_E, -(nCurrentPos) * (nCurrentPos))));
}

inline double CosineImpactFunc(
    double nStartVal, double nCurrentPos, double nRange)
{
    return (nStartVal * 0.5 * (cos(M_PI * nCurrentPos / nRange) + 1.0));
}

}  // namespace ChaoVis
