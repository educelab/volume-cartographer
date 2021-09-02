#pragma once

#include "vc/core/filesystem.hpp"
#include "vc/core/types/UVMap.hpp"

namespace volcart::io
{
/** @brief Write a UVMap in the custom .uvm archival format */
void WriteUVMap(const filesystem::path& path, const UVMap& uvMap);

/** @brief Read a UVMap from the custom .uvm archival format */
auto ReadUVMap(const filesystem::path& path) -> UVMap;
}  // namespace volcart::io
