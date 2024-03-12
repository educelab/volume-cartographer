#pragma once

/** @file */

#include <memory>

#include "vc/core/types/VolumetricMask.hpp"
#include "vc/texturing/TexturingAlgorithm.hpp"

namespace volcart::texturing
{

/**
 * @brief Generate a Texture using the thickness of the segmented layer
 *
 * For each point, project bi-directionally along the surface normal until
 * finding the first positive and negative points <b>not</b> in the mask.
 * Thickness is the Euclidean distance between these two points. If
 * setNormalizeOutput is true (default), the returned image will be normalized
 * between [0, 1]. Otherwise, raw distances will be returned.
 *
 * Returned image is single-channel, 32-bit floating point.
 *
 * @ingroup Texture
 */
class ThicknessTexture : public TexturingAlgorithm
{
public:
    /** Pointer type */
    using Pointer = std::shared_ptr<ThicknessTexture>;

    /** Make shared pointer */
    static auto New() -> Pointer;

    /** Default constructor */
    ThicknessTexture() = default;
    /** Default destructor */
    ~ThicknessTexture() override = default;
    /** Default copy constructor */
    ThicknessTexture(ThicknessTexture&) = default;
    /** Default move constructor */
    ThicknessTexture(ThicknessTexture&&) = default;
    /** Default copy operator */
    auto operator=(const ThicknessTexture&) -> ThicknessTexture& = default;
    /** Default move operator */
    auto operator=(ThicknessTexture&&) -> ThicknessTexture& = default;

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

    /** @copydetails setSamplingInterval(double) */
    [[nodiscard]] auto samplingInterval() const -> double;

    /**
     * @brief Normalize the output image
     *
     * If true (default), normalize the output image between [0, 1]. Otherwise,
     * return the raw distance values.
     */
    void setNormalizeOutput(bool b);

    /** @copydetails setNormalizeOutput(bool) */
    [[nodiscard]] auto normalizeOutput() const -> bool;

    /** @brief Set the VolumetricMask */
    void setVolumetricMask(const VolumetricMask::Pointer& m);

    /** @brief Get the VolumetricMask */
    [[nodiscard]] auto volumetricMask() const -> VolumetricMask::Pointer;

    /** @brief Compute the result */
    auto compute() -> Texture override;

private:
    /** Volumetric mask */
    VolumetricMask::Pointer mask_;
    /** Sampling interval */
    double interval_{1.0};
    /** Normalize output */
    bool normalize_{true};
};
}  // namespace volcart::texturing
