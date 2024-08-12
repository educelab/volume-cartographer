#include "vc/core/types/DiskBasedObjectBaseClass.hpp"

using namespace volcart;

namespace fs = volcart::filesystem;

static const fs::path METADATA_FILE = "meta.json";

// Load file from disk
DiskBasedObjectBaseClass::DiskBasedObjectBaseClass(
    volcart::filesystem::path path)
    : path_(std::move(path))
{
    metadata_ = volcart::Metadata(path_ / METADATA_FILE);
}

// Create new file on disk
DiskBasedObjectBaseClass::DiskBasedObjectBaseClass(
    fs::path path, Identifier uuid, std::string name)
    : path_(std::move(path))
{
    metadata_.setPath((path_ / METADATA_FILE));
    metadata_.set("uuid", uuid);
    metadata_.set("name", name);
}