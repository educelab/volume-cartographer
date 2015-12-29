// checkPtInTriangleUtil.h
// Chao 2015 April
#ifndef _CHECKPTINTRIANGLEUTIL_H_
#define _CHECKPTINTRIANGLEUTIL_H_

namespace checkPtInTriangleUtil {
    
// Check if the point is within the angle (note it could also be in the reverse direction)
// A is the starting point, AB and AC are the edges
inline bool IsPtBetweenVecs( const cv::Vec3d &nPt,
                             const cv::Vec3d &nA,
                             const cv::Vec3d &nB,
                             const cv::Vec3d &nC )
{
    cv::Vec3d aAB = nB - nA;
    cv::Vec3d aAC = nC - nA;
    cv::Vec3d aAP = nPt - nA;
    return( aAB.cross(aAP)[2] * aAC.cross(aAP)[2] < 0 );
}

// Check is the point is inside the triangle
// Barycentric technique taken from here: http://www.blackpawn.com/texts/pointinpoly/
inline bool IsPtInTriangle( const cv::Vec3d &nPt,
                            const cv::Vec3d &nA,
                            const cv::Vec3d &nB,
                            const cv::Vec3d &nC )
{
    cv::Vec3d aAB = nB - nA;
    cv::Vec3d aAC = nC - nA;
    cv::Vec3d aAP = nPt - nA;

    double dot00 = aAC.dot(aAC);
    double dot01 = aAC.dot(aAB);
    double dot02 = aAC.dot(aAP);
    double dot11 = aAB.dot(aAB);
    double dot12 = aAB.dot(aAP);

    double invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
    double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

    return ( u >= 0 && v >= 0 && u + v < 1 );
}

} // checkPtInTriangleUtil

#endif // _CHECKPTINTRIANGLEUTIL_H_
