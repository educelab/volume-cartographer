//
// Created by Seth Parker on 12/28/15.
//

#ifndef VC_COMPOSITETEXTUREV2_H
#define VC_COMPOSITETEXTUREV2_H

#include "vc_defines.h"
#include "vc_datatypes.h"
#include "volumepkg.h"

#include <opencv2/opencv.hpp>

#include "UPointMapping.h"
#include "checkPtInTriangleUtil.h"
#include "texturingUtils.h"

namespace volcart {
namespace texturing {

    class compositeTextureV2 {
    public:
        struct cellInfo {
            std::vector<cv::Vec3d> Pts2D;
            std::vector<cv::Vec3d> Pts3D;
        };

        compositeTextureV2( VC_MeshType::Pointer inputMesh,
                            VolumePkg& volpkg,
                            UVMap uvMap,
                            double radius,
                            int width,
                            int height,
                            VC_Composite_Option method = VC_Composite_Option::NonMaximumSuppression,
                            VC_Direction_Option direction = VC_Direction_Option::Bidirectional );

        volcart::Texture texture() { return _texture; };
    private:
        int _process();
        int _generateCellInfo();
        cv::Vec3d _BarycentricCoord( const cv::Vec3d &nXYZ, const cv::Vec3d &nA, const cv::Vec3d &nB, const cv::Vec3d &nC );
        cv::Vec3d _CartesianCoord  ( const cv::Vec3d &nUVW, const cv::Vec3d &nA, const cv::Vec3d &nB, const cv::Vec3d &nC );

        // Variables
        VC_MeshType::Pointer _input;
        VolumePkg& _volpkg;
        int _width;
        int _height;
        double _radius;
        VC_Composite_Option _method;
        VC_Direction_Option _direction;

        UVMap _uvMap;
        Texture _texture;

        std::vector< cellInfo > _cellInformation;
    };

}
}

#endif //VC_COMPOSITETEXTUREV2_H
