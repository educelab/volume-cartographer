//
// Created by Melissa Shankle on 12/16/15.
//

#include "Sphere.h"
#include "../vc_defines.h"

namespace volcart {
    namespace shapes {

        Sphere::Sphere(float radius, int recursionLevel) {

            // create 12 vertices of a icosahedron
            double t = ( 1.0 + sqrt(5.0) ) / 2.0;

            // must make everything on the unit sphere, so divide by length
            double length = sqrt(1 + t * t);

            _add_vertex(-1/length,  t/length, 0);
            _add_vertex( 1/length,  t/length, 0);
            _add_vertex(-1/length, -t/length, 0);
            _add_vertex( 1/length, -t/length, 0);

            _add_vertex( 0, -1/length,  t/length);
            _add_vertex( 0,  1/length,  t/length);
            _add_vertex( 0, -1/length, -t/length);
            _add_vertex( 0,  1/length, -t/length);

            _add_vertex( t/length, 0, -1/length);
            _add_vertex( t/length, 0,  1/length);
            _add_vertex(-t/length, 0, -1/length);
            _add_vertex(-t/length, 0,  1/length);


            // create 20 triangles of the icosahedron
            std::vector<VC_Cell> _temp_cells;

            // 5 faces around point 0
            _temp_cells.push_back( VC_Cell(0, 11,   5) );
            _temp_cells.push_back( VC_Cell(0,  5,   1) );
            _temp_cells.push_back( VC_Cell(0,  1,   7) );
            _temp_cells.push_back( VC_Cell(0,  7,  10) );
            _temp_cells.push_back( VC_Cell(0, 10,  11) );

            // 5 adjacent faces
            _temp_cells.push_back( VC_Cell( 1,  5, 9) );
            _temp_cells.push_back( VC_Cell( 5, 11, 4) );
            _temp_cells.push_back( VC_Cell(11, 10, 2) );
            _temp_cells.push_back( VC_Cell(10,  7, 6) );
            _temp_cells.push_back( VC_Cell( 7,  1, 8) );

            // 5 faces around point 3
            _temp_cells.push_back( VC_Cell(3, 9, 4) );
            _temp_cells.push_back( VC_Cell(3, 4, 2) );
            _temp_cells.push_back( VC_Cell(3, 2, 6) );
            _temp_cells.push_back( VC_Cell(3, 6, 8) );
            _temp_cells.push_back( VC_Cell(3, 8, 9) );

            // 5 adjacent faces
            _temp_cells.push_back( VC_Cell(4, 9,  5) );
            _temp_cells.push_back( VC_Cell(2, 4, 11) );
            _temp_cells.push_back( VC_Cell(6, 2, 10) );
            _temp_cells.push_back( VC_Cell(8, 6,  7) );
            _temp_cells.push_back( VC_Cell(9, 8,  1) );


            // refine triangles
            for( int i = 0; i < recursionLevel; i++) {
                std::vector<VC_Cell> _temp_cells2;

                for( auto cell = _temp_cells.begin(); cell != _temp_cells.end(); ++cell ) {
                    // replace using 4 triangles

                    int a = _midpoint( cell->v1, cell->v2 );
                    int b = _midpoint( cell->v2, cell->v3 );
                    int c = _midpoint( cell->v3, cell->v1 );

                    _temp_cells2.push_back( VC_Cell(cell->v1, a, c) );
                    _temp_cells2.push_back( VC_Cell(cell->v2, b, a) );
                    _temp_cells2.push_back( VC_Cell(cell->v3, c, b) );
                    _temp_cells2.push_back( VC_Cell(a, b, c) );

                }
                _temp_cells = _temp_cells2;
            }

            // add triangles to mesh
            for( auto cell = _temp_cells.begin(); cell != _temp_cells.end(); ++cell ) {
                _add_cell( cell->v1, cell->v2, cell->v3 );
            }

            _orderedPoints = false;

        } // Constructor

        int Sphere::_midpoint(int p1, int p2){

            // Generate unique id
            bool firstIsSmaller = p1 < p2;
            int smallerIndex = firstIsSmaller ? p1 : p2;
            int greaterIndex = firstIsSmaller ? p2 : p1;
            std::string key = std::to_string(smallerIndex) + "-" + std::to_string(greaterIndex);

            // Find unique id and return if cached
            auto found = _indexCache.find(key);
            if ( found != _indexCache.end() )
                return found->second;

            // calculate
            double midX, midY, midZ;

            midX = (_points[p1].x + _points[p2].x) / 2;
            midY = (_points[p1].y + _points[p2].y) / 2;
            midZ = (_points[p1].z + _points[p2].z) / 2;;

            // ensure unit sphere by dividing each point by length
            double length = sqrt(midX * midX + midY * midY + midZ * midZ);
            double x = midX / length;
            double y = midY / length;
            double z = midZ / length;

            // Add to points list and cache
            _add_vertex( x, y, z );
            _indexCache.insert( { key, _points.size() - 1 } );

            return (_points.size() - 1);
        }  // _midpoint
    } // shapes
} // volcart