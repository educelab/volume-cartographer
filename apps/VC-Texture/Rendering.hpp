#pragma once

#include "vc/core/types/ITKMesh.hpp"
#include "vc/core/types/Metadata.hpp"
#include "vc/core/types/Texture.hpp"

/**
 * @class Rendering
 * @author Seth Parker
 * @date 6/9/16
 *
 * @brief Holds the final results of a render pipeline
 *
 * After segmentation, the segmentation mesh is often cleaned and resampled,
 * flattened, and textured. The final result of this pipeline, a Rendering, is
 * largely dependent upon the algorithmic decisions made at each step in the
 * process. This class provides a mechanism for grouping these results and
 * their associated metadata into a single data structure.
 *
 * @warning This class's intended purpose is now supplied by volcart::Render.
 * Please use that class instead for storing the final results of a rendering
 * pipeline.
 */
class Rendering
{
public:
    /**@{*/
    /** @brief Default constructor */
    Rendering();
    /**@}*/

    /**@{*/
    /** @brief Get the Metadata for the Rendering */
    volcart::Metadata metadata() const { return metadata_; }

    /** @brief Get the Rendering ID */
    std::string id() const { return metadata_.get<std::string>("id"); }
    /**@}*/

    /**@{*/
    /** @brief Assign the Texture */
    void setTexture(volcart::Texture texture) { texture_ = std::move(texture); }

    /** @brief Get the Texture */
    volcart::Texture getTexture() const { return texture_; }
    /**@}*/

    /**@{*/
    /** @brief Set the ITKMesh */
    void setMesh(const volcart::ITKMesh::Pointer& mesh) { mesh_ = mesh; }

    /** @brief Get the ITKMesh */
    volcart::ITKMesh::Pointer getMesh() const { return mesh_; }
    /**@}*/

private:
    /** Metadata */
    volcart::Metadata metadata_;
    /** Texture */
    volcart::Texture texture_;
    /** ITKMesh */
    volcart::ITKMesh::Pointer mesh_;
};
