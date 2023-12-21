#pragma once

#include <boost/program_options.hpp>

/** Get the flattening/UV options */
auto GetUVOpts() -> boost::program_options::options_description;

/** Get the generic texture filtering options */
auto GetFilteringOpts() -> boost::program_options::options_description;

/** Get the Composite Texture options */
auto GetCompositeOpts() -> boost::program_options::options_description;

/** Get the Integral Texture options */
auto GetIntegralOpts() -> boost::program_options::options_description;

/** Get the Thickness Texture options */
auto GetThicknessOpts() -> boost::program_options::options_description;

/** Available neighborhood generators */
enum class Shape { Line = 0, Cuboid };

/** Available texturing algorithms */
enum class Method { Composite = 0, Intersection, Integral, Thickness };