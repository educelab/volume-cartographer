// volcart::texturing::compositeTexture
// Generate a Texture object using one of the various filters in texturingUtils.h.
// Created by Seth Parker on 10/20/15.

#ifndef VC_COMPOSITETEXTURE_H
#define VC_COMPOSITETEXTURE_H

#include "vc_defines.h"
#include "vc_datatypes.h"
#include "volumepkg.h"

#include "texturingUtils.h"
#include "simpleUV.h"

namespace volcart {
    namespace texturing {
        class compositeTexture {
        public:
            compositeTexture(VC_MeshType::Pointer inputMesh,
                             VolumePkg& volpkg,
                             int output_w,
                             int output_h,
                             double radius,
                             VC_Composite_Option compositeMethod,
                             VC_Direction_Option compositeDirection);

            Texture texture() { return _texture; };
        private:
            int _process();

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
        }; // compositeTexture
    } // volcart
}// texture

#endif //VC_COMPOSITETEXTURE_H
