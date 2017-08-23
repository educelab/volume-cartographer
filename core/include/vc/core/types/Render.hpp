#pragma once

#include "vc/core/types/DiskBasedObjectBaseClass.hpp"

namespace volcart
{

/**
 * @class Render
 * @author Seth Parker
 *
 * @brief Dender data
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
    explicit Render(boost::filesystem::path path);

    /** @brief Make a new Render in a directory */
    Render(boost::filesystem::path path, Identifier uuid, std::string name);

    /** @copydoc Render(boost::filesystem::path path) */
    static Pointer New(boost::filesystem::path path);

    /** @copydoc Render(boost::filesystem::path path, Identifier uuid,
     * std::string name) */
    static Pointer New(
        boost::filesystem::path path, Identifier uuid, std::string name);
};
}