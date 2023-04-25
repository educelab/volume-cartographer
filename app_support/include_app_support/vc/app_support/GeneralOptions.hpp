#pragma once

/** @file */

#include <boost/program_options.hpp>

/** Get general options related to all rendering */
auto GetGeneralOpts() -> boost::program_options::options_description;

/** Get general options related to mesh IO configuration */
auto GetMeshIOOpts() -> boost::program_options::options_description;