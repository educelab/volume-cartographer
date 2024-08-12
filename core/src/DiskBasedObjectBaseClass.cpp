#include <utility>

#include "vc/core/types/DiskBasedObjectBaseClass.hpp"

using namespace volcart;

namespace fs = filesystem;

static const fs::path METADATA_FILE = "meta.json";

DiskBasedObjectBaseClass::Identifier DiskBasedObjectBaseClass::id() const
{
    return metadata_.get<std::string>("uuid").value();
}

auto DiskBasedObjectBaseClass::path() const -> filesystem::path
{
    return path_;
}

auto DiskBasedObjectBaseClass::name() const -> std::string
{
    return metadata_.get<std::string>("name").value();
}

void DiskBasedObjectBaseClass::setName(std::string n)
{
    metadata_.set("name", std::move(n));
}
void DiskBasedObjectBaseClass::saveMetadata() const { metadata_.save(); }

// Load file from disk
DiskBasedObjectBaseClass::DiskBasedObjectBaseClass(filesystem::path path)
    : path_(std::move(path))
{
    metadata_ = Metadata(path_ / METADATA_FILE);
}

// Create new file on disk
DiskBasedObjectBaseClass::DiskBasedObjectBaseClass(
    fs::path path, Identifier uuid, std::string name)
    : path_(std::move(path))
{
    metadata_.setPath(path_ / METADATA_FILE);
    metadata_.set("uuid", std::move(uuid));
    metadata_.set("name", std::move(name));
}