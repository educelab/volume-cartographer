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
        volcart::Texture compositeTexture( VC_MeshType::Pointer inputMesh,
                                           VolumePkg volpkg,
                                           int output_w,
                                           int output_h,
                                           double searchMajorRadius,
                                           VC_Composite_Option compositeMethod,
                                           VC_Direction_Option compositeDirection);
    } // volcart
}// texture

#endif //VC_COMPOSITETEXTURE_H
