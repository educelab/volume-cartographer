//
// Created by Seth Parker on 6/9/16.
// Angle-based Flattening implementation based on the same from Blender

#ifndef VC_ABF_H
#define VC_ABF_H

#include <iostream>

#include "vc_defines.h"
#include "vc_datatypes.h"
#include "deepCopy.h"

namespace volcart {
    namespace texturing {

      class abf {
      public:
          ///// Constructors/Destructors /////
          abf(){};
          abf( VC_MeshType::Pointer input );
          ~abf(){};

          ///// Access Functions /////
          // Set inputs
          void setMesh( VC_MeshType::Pointer input );

          // Get outputs
          VC_MeshType::Pointer getMesh();
          volcart::UVMap getUVMap();

          ///// Process /////
          void compute();
      private:

          const VC_MeshType::Pointer _mesh;
      };

    }// texturing
}//volcart

#endif //VC_ABF_H
