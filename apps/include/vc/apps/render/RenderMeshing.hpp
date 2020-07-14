#pragma once

#include <boost/program_options.hpp>

#include "vc/core/types/ITKMesh.hpp"

/** Get meshing options */
boost::program_options::options_description GetMeshingOpts();

/** When to smooth the mesh */
enum class SmoothOpt { Off = 0, Before, After, Both };

/** Perform mesh resampling and smoothing */
volcart::ITKMesh::Pointer ResampleMesh(const volcart::ITKMesh::Pointer& m);