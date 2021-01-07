#pragma once

#include <memory>

#include "vc/core/types/VolumetricMask.hpp"
#include "vc/texturing/TexturingAlgorithm.hpp"

namespace volcart
{
namespace texturing
{

/**
 * @brief Texture using the thickness of the segmented layer
 *
 * For each point, project bi-directionally along the surface normal until
 * finding the first positive and negative points <b>not</b> in the mask.
 * Thickness is the Euclidean distance between these two points. If
 * setNormalizeOutput is true (default), the returned image will be normalized
 * between [0, 1]. Otherwise, raw distances will be returned.
 *
 * Returned image is single-channel, 32-bit floating point.
 */
class ThicknessTexture : public TexturingAlgorithm
{
public:
    /** Pointer type */
    using Pointer = std::shared_ptr<ThicknessTexture>;

    /** Static New function for all constructors of T */
    template <typename... Args>
    static Pointer New(Args... args)
    {
        return std::make_shared<ThicknessTexture>(std::forward<Args>(args)...);
    }

    /** Destructor */
    ~ThicknessTexture() override = default;

    /**
     * @brief Set the sampling interval
     *
     * In Volume units, how frequently to check whether a sample is part of the
     * VolumetricMask. In theory, smaller values return more accurate results at
     * the expense of longer runtimes. However, the VolumetricMask does not
     * support sub-voxel mask precision, so improvements will be minimal for
     * interval values < 1.
     */
    void setSamplingInterval(double i);

    /**
     * @brief Normalize the output image
     *
     * If true (default), normalize the output image between [0, 1]. Otherwise,
     * return the raw distance values.
     */
    void setNormalizeOutput(bool b);

    /** @brief Set the VolumetricMask */
    void setVolumetricMask(const VolumetricMask::Pointer& m);

    /** @brief Compute the result */
    Texture compute() override;

private:
    /** Volumetric mask */
    VolumetricMask::Pointer mask_;
    /** Sampling interval */
    double interval_{1.0};
    /** Normalize output */
    bool normalize_{true};
};
}  // namespace texturing
}  // namespace volcart