#include "vc/core/types/Render.hpp"

#include <utility>

using namespace volcart;

namespace fs = volcart::filesystem;
using namespace smgl;

using GraphPtr = std::shared_ptr<Graph>;

// Load a Render directory from disk
// Reads and verifies metadata
Render::Render(fs::path path) : DiskBasedObjectBaseClass(std::move(path))
{
    if (metadata_.get<std::string>("type") != "render") {
        throw std::runtime_error("File not of type: render");
    }
}

// Make a new Render file on disk
Render::Render(fs::path path, Identifier uuid, std::string name)
    : DiskBasedObjectBaseClass(
          std::move(path), std::move(uuid), std::move(name))
{
    metadata_.set("type", "render");

    // Create a graph
    graph_ = std::make_shared<Graph>();
    graph_->setEnableCache(true);
    graph_->setCacheType(CacheType::Adjacent);
    graph_->setCacheFile(path_ / "graph.json");
    Graph::Save(path_ / "graph.json", *graph_, true);
    metadata_.set("graph", "graph.json");

    metadata_.save();
}

// Load a Render from disk, return a pointer
auto Render::New(fs::path path) -> Render::Pointer
{
    return std::make_shared<Render>(path);
}

// Make a new Render on disk, return a pointer
auto Render::New(fs::path path, std::string uuid, std::string name)
    -> Render::Pointer
{
    return std::make_shared<Render>(path, uuid, name);
}

auto Render::graph() const -> GraphPtr
{
    // Lazy load the graph
    if (not graph_ and metadata_.hasKey("graph")) {
        auto g = Graph::Load(path_ / metadata_.get<std::string>("graph"));
        graph_ = std::make_shared<Graph>(g);
    }
    return graph_;
}
