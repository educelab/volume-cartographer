//
// Created by Seth Parker on 6/9/16.
//

#include "abf.h"


namespace volcart {
    namespace texturing {

      ///// Process //////
      void abf::compute()
      {
        // 1) Get id's for vertices, faces, edges, angles
        //    - Determine whether vertices are interior. Do I need to compute some boundary before this?

        // 1.5) Setup system defaults

        /* 2) Compute original angles
            - Sum angles for every vertex edge calc "scale"
            - Scale beta by "scale" and set this as alpha & beta for that edge
        */

        /* 3) If there are interior vertices:
            loop until max iterations reached - fallback to lscm if reached
              compute gradient for the mesh
              break if the grad is below the error limit
              invert the matrix - fallback to lscm if this fails
              compute sines
        */
      }

      // Setup
        // Sorting
        // Defaults
        // Angles

      // Minimization Loop
        // compute gradient
        // invert the matrix *All the hard work*

    } // texturing
} // volcart