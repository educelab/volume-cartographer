#include "vc/core/types/Render.hpp"

using namespace volcart;

namespace fs = boost::filesystem;

// Load a Render directory from disk
// Reads and verifies metadata
Render::Render(fs::path path) : DiskBasedObjectBaseClass(path)
{
    if (metadata_.get<std::string>("type") != "render") {
        throw std::runtime_error("File not of type: render");
    }
}

// Make a new Render file on disk
Render::Render(fs::path path, Identifier uuid, std::string name)
    : DiskBasedObjectBaseClass(path, uuid, name)
{
    metadata_.set("type", "render");
    metadata_.save();
}

// Load a Render from disk, return a pointer
Render::Pointer Render::New(fs::path path)
{
    return std::make_shared<Render>(path);
}

// Make a new Render on disk, return a pointer
Render::Pointer Render::New(fs::path path, std::string uuid, std::string name)
{
    return std::make_shared<Render>(path, uuid, name);
}