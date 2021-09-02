#pragma once

/** @file */

#include <memory>

#include <smgl/Graph.hpp>

#include "vc/core/filesystem.hpp"
#include "vc/core/types/DiskBasedObjectBaseClass.hpp"

namespace volcart
{

/**
 * @class Render
 * @author Seth Parker
 *
 * @brief Render data
 *
 * Provides access to Render information stored on disk, usually inside of
 * a VolumePkg.
 *
 * @ingroup Types
 */
class Render : public DiskBasedObjectBaseClass
{
public:
    /** Shared pointer type */
    using Pointer = std::shared_ptr<Render>;

    /** @brief Load a Render from file */
    explicit Render(filesystem::path path);

    /** @brief Make a new Render in a directory */
    Render(filesystem::path path, Identifier uuid, std::string name);

    /** @copydoc Render(volcart::filesystem::path path) */
    static auto New(filesystem::path path) -> Pointer;

    /** @copydoc Render(volcart::filesystem::path path, Identifier uuid,
     * std::string name) */
    static auto New(filesystem::path path, Identifier uuid, std::string name)
        -> Pointer;

    /**
     * @brief Get the stored render graph
     *
     * If not already loaded, this function loads and returns the computational
     * graph for this render. All nodes used in the graph should be registered
     * with the smgl serialization system before calling this function. Nodes
     * provided by Volume Cartographer can be registered with RegisterNodes().
     *
     * @throws smgl::unknown_identifier If an unregistered node is encountered
     */
    [[nodiscard]] auto graph() const -> std::shared_ptr<smgl::Graph>;

private:
    /** Render graph */
    mutable std::shared_ptr<smgl::Graph> graph_;
};
}  // namespace volcart
