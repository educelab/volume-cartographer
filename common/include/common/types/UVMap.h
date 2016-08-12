// VC UV Map
// Object to store UV mappings for points in a mesh.
// Created by Seth Parker on 10/20/15.

// Internally, all mappings are stored relative to the "top-left" of the 2D mapping space. get() & set() transform
// values to and from this relative range based on the value of _origin. By setting _origin prior to insertion and
// again prior to retrieval, mappings can be inserted relative to one origin but retrieved relative to another.

#ifndef VC_UVMAP_H
#define VC_UVMAP_H

#include "common/vc_defines.h"
#include <opencv2/opencv.hpp>
#include <unordered_map>

namespace volcart {

    class UVMap {
    public:
        UVMap() { _origin = VC_ORIGIN_TOP_LEFT; };
        UVMap(VC_Origin o) { _origin = o; };

        size_t size() { return _map.size(); };
        bool empty() { return _map.empty(); };

        // Get and set origin
        VC_Origin  origin() { return _origin; };
        void origin(VC_Origin o) { _origin = o; };

        // Set the uv mapping for point p_id
        void set(double p_id, cv::Vec2d uv) {
            cv::Vec2d transformed;
            cv::absdiff(uv, _origin, transformed); // transform to be relative to top-left
            _map.insert( {p_id, transformed} );
        };

        // Get the uv mapping for point p_id
        cv::Vec2d get(double p_id) {
            auto it = _map.find(p_id);
            if ( it != _map.end() ) {
                cv::Vec2d transformed;
                cv::absdiff(it->second, _origin, transformed); // transform to be relative to _origin
                return transformed;
            }
            else return VC_UVMAP_NULL_MAPPING;
        };

        // Ratio information
        VC_Ratio ratio() { return _ratio; };
        void ratio( double a ) { _ratio.aspect = a; };
        void ratio( double w, double h ) {
            _ratio.width = w;
            _ratio.height = h;
            _ratio.aspect = w / h;
        };

    private:
        std::unordered_map<double, cv::Vec2d> _map; // holds the mapping
        cv::Vec2d _origin; // origin inserted and retrieved points are relative to
        VC_Ratio _ratio;
    };

} // volcart

#endif //VC_UVMAP_H
